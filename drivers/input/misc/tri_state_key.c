/*
<<<<<<< HEAD
 * drivers/input/misc/tri_state_key.c
 *
 * Copyright (C) 2018, Sultanxda <sultanxda@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#define DRIVER_NAME "tri-state-key"
#define NR_STATES (3)

enum key_vals {
	KEYCODE_TOTAL_SILENCE = 600,
	KEYCODE_MODE_ALARMS_ONLY,
	KEYCODE_MODE_PRIORITY_ONLY,
	KEYCODE_MODE_NONE,
	KEYCODE_MODE_MAX
};

static const char *const proc_names[] = {
	"keyCode_top", "keyCode_middle", "keyCode_bottom"
};

struct tristate_data {
	struct device *dev;
	struct input_dev *input;
	struct mutex irq_lock;
	enum key_vals key_codes[NR_STATES];
	int key_gpios[NR_STATES];
	uint8_t curr_state;
};

static struct tristate_data *tdata_g;

static void send_input(struct tristate_data *t, int keycode)
{
	input_report_key(t->input, keycode, 1);
	input_sync(t->input);
	input_report_key(t->input, keycode, 0);
	input_sync(t->input);
}

static void tristate_process_state(struct tristate_data *t)
{
	unsigned long key_state = 0;
	int i;

	/* Store the GPIO values of the keys as a bitmap */
	for (i = 0; i < NR_STATES; i++)
		key_state |= !!gpio_get_value(t->key_gpios[i]) << i;

	/* Spurious interrupt; at least one of the pins should be zero */
	if (key_state == 0x7)
		return;

	/* Redundant interrupt */
	if (key_state == t->curr_state)
		return;

	t->curr_state = key_state;

	/*
	 * Find the first bit set to zero; this is the current position. Bit 0
	 * corresponds to the top position, bit 1 corresponds to the middle
	 * position, and bit 2 corresponds to the bottom position.
	 */
	i = ffz(key_state);
	send_input(t, t->key_codes[i]);
}

static irqreturn_t tristate_irq_handler(int irq, void *dev_id)
{
	struct tristate_data *t = dev_id;

	/* This handler is shared between three interrupt lines */
	mutex_lock(&t->irq_lock);
	tristate_process_state(t);
	mutex_unlock(&t->irq_lock);

	return IRQ_HANDLED;
}

static int keycode_get_idx(const char *filename)
{
	int i;

	for (i = 0; i < NR_STATES; i++) {
		if (!strcmp(proc_names[i], filename))
			break;
	}

	return i;
}

static int keycode_show(struct seq_file *seq, void *offset)
{
	struct tristate_data *t = tdata_g;
	int idx = (int)(long)seq->private;

	seq_printf(seq, "%d\n", t->key_codes[idx]);
	return 0;
}

static ssize_t tristate_keycode_write(struct file *file,
	const char __user *page, size_t count, loff_t *offset)
{
	struct tristate_data *t = tdata_g;
	char buf[sizeof("600")];
	int data, idx;

	memset(buf, 0, sizeof(buf));

	if (copy_from_user(buf, page, min_t(int, count, 3))) {
		dev_err(t->dev, "Failed to read procfs input\n");
		return count;
	}

	if (kstrtoint(buf, 10, &data) < 0)
		return count;

	if (data < KEYCODE_TOTAL_SILENCE || data >= KEYCODE_MODE_MAX)
		return count;

	idx = keycode_get_idx(file->f_path.dentry->d_iname);

	mutex_lock(&t->irq_lock);
	t->key_codes[idx] = data;
	if (!(t->curr_state & BIT(idx)))
		send_input(t, data);
	mutex_unlock(&t->irq_lock);

	return count;
}

static int tristate_keycode_open(struct inode *inode, struct file *file)
{
	int idx = keycode_get_idx(file->f_path.dentry->d_iname);

	/* Pass the index to keycode_show */
	return single_open(file, keycode_show, (void *)(long)idx);
}

static const struct file_operations tristate_keycode_proc = {
	.owner		= THIS_MODULE,
	.open		= tristate_keycode_open,
	.read		= seq_read,
	.write		= tristate_keycode_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int tristate_parse_dt(struct tristate_data *t)
{
	struct device_node *np;
	char prop_name[sizeof("tristate,gpio_key#")];
	int i;

	np = t->dev->of_node;
	if (!np)
		return -EINVAL;

	for (i = 0; i < NR_STATES; i++) {
		sprintf(prop_name, "tristate,gpio_key%d", i + 1);

		t->key_gpios[i] = of_get_named_gpio(np, prop_name, 0);
		if (!gpio_is_valid(t->key_gpios[i])) {
			dev_err(t->dev, "Invalid %s property\n", prop_name);
			return -EINVAL;
		}
	}

	return 0;
}

static int tristate_register_irqs(struct tristate_data *t)
{
	char label[sizeof("tristate_key#-int")];
	int i, j, key_irqs[NR_STATES], ret;

	/* Get the IRQ numbers corresponding to the GPIOs */
	for (i = 0; i < NR_STATES; i++) {
		key_irqs[i] = gpio_to_irq(t->key_gpios[i]);
		if (key_irqs[i] < 0) {
			dev_err(t->dev, "Invalid IRQ (%d) for GPIO%d\n",
				key_irqs[i], i + 1);
			return -EINVAL;
		}
	}

	for (i = 0; i < NR_STATES; i++) {
		sprintf(label, "tristate_key%d-int", i + 1);

		ret = gpio_request(t->key_gpios[i], label);
		if (ret < 0) {
			dev_err(t->dev, "Failed to request GPIO%d, ret: %d\n",
				i + 1, ret);
			goto free_gpios;
		}

		ret = gpio_direction_input(t->key_gpios[i]);
		if (ret < 0) {
			dev_err(t->dev, "Failed to set GPIO%d dir, ret: %d\n",
				i + 1, ret);
			goto free_gpios;
		}

		sprintf(label, "tristate_key%d", i + 1);

		ret = request_threaded_irq(key_irqs[i], NULL,
				tristate_irq_handler,
				IRQF_TRIGGER_FALLING | IRQF_ONESHOT, label, t);
		if (ret < 0) {
			dev_err(t->dev, "Failed to register %s IRQ, ret: %d\n",
				label, ret);
			goto free_irqs;
		}
	}

	for (i = 0; i < NR_STATES; i++)
		enable_irq_wake(key_irqs[i]);

	return 0;

free_irqs:
	for (j = i; j--;)
		free_irq(key_irqs[j], t);
	i = NR_STATES;
free_gpios:
	for (j = i; j--;)
		gpio_free(t->key_gpios[j]);
	return -EINVAL;
}

static void tristate_create_procfs(void)
{
	struct proc_dir_entry *proc_dir;
	int i;

	proc_dir = proc_mkdir("tri-state-key", NULL);
	if (!proc_dir)
		return;

	for (i = 0; i < NR_STATES; i++)
		proc_create_data(proc_names[i], 0644, proc_dir,
			&tristate_keycode_proc, NULL);
}

static int tristate_probe(struct platform_device *pdev)
{
	struct tristate_data *t;
	int i, ret;

	t = kzalloc(sizeof(*t), GFP_KERNEL);
	if (!t)
		return -ENOMEM;

	t->dev = &pdev->dev;
	tdata_g = t;

	ret = tristate_parse_dt(t);
	if (ret)
		goto free_t;

	t->input = input_allocate_device();
	if (!t->input) {
		dev_err(t->dev, "Failed to allocate input device\n");
		ret = -ENOMEM;
		goto free_t;
	}

	t->input->name = DRIVER_NAME;
	t->input->dev.parent = t->dev;
	input_set_drvdata(t->input, t);

	set_bit(EV_KEY, t->input->evbit);
	set_bit(KEYCODE_TOTAL_SILENCE, t->input->keybit);
	set_bit(KEYCODE_MODE_ALARMS_ONLY, t->input->keybit);
	set_bit(KEYCODE_MODE_PRIORITY_ONLY, t->input->keybit);
	set_bit(KEYCODE_MODE_NONE, t->input->keybit);

	ret = input_register_device(t->input);
	if (ret)
		goto free_input;

	/* Initialize with default keycodes */
	for (i = 0; i < NR_STATES; i++)
		t->key_codes[i] = KEYCODE_TOTAL_SILENCE + i;

	/* Process initial state */
	tristate_process_state(t);

	mutex_init(&t->irq_lock);
	ret = tristate_register_irqs(t);
	if (ret)
		goto input_unregister;

	tristate_create_procfs();
	return 0;

input_unregister:
	input_unregister_device(t->input);
free_input:
	input_free_device(t->input);
free_t:
	kfree(t);
	return ret;
}

static const struct of_device_id tristate_of_match[] = {
	{ .compatible = "oneplus,tri-state-key", },
	{ },
};

static struct platform_driver tristate_driver = {
	.probe	= tristate_probe,
	.driver	= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = tristate_of_match,
	},
};

static int __init tristate_init(void)
{
	return platform_driver_register(&tristate_driver);
}
device_initcall(tristate_init);
=======
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>

#include <linux/regulator/consumer.h>

#include <linux/timer.h>

#define DRV_NAME	"tri-state-key"

/*
	        KEY1(GPIO1)	KEY2(GPIO92)
1脚和4脚连接	0	            1         | MUTE
2脚和5脚连接	1	            1         | Do Not Disturb
4脚和3脚连接	1	            0         | Normal

*/
typedef enum {
    MODE_UNKNOWN,
	MODE_MUTE,
	MODE_DO_NOT_DISTURB,
	MODE_NORMAL,
	MODE_MAX_NUM
} tri_mode_t;

struct switch_dev_data {
	//tri_mode_t last_type;
	//tri_mode_t mode_type;
	//int switch_enable;
	int irq_key3;
	int irq_key2;
	int irq_key1;
	int key1_gpio;//key1 gpio34
	int key2_gpio;//key2 gpio77
	int key3_gpio;

	struct regulator *vdd_io;
	//bool power_enabled;

	struct work_struct work;
	struct switch_dev sdev;
	struct device *dev;
	//struct input_dev *input;

	struct timer_list s_timer;
	struct pinctrl * key_pinctrl;
	struct pinctrl_state * set_state;

};

static struct switch_dev_data *switch_data;
static DEFINE_MUTEX(sem);
#if 0
static int set_gpio_by_pinctrl(void)
{
    printk(KERN_ERR "tristate_key set_gpio_by_pinctrl. \n");
    return pinctrl_select_state(switch_data->key_pinctrl, switch_data->set_state);
}
#endif
static void switch_dev_work(struct work_struct *work)
{

	int key1,key2,key3;
	//pr_err("%s  gpio_get_value(%d)=%d\n",__func__,switch_data->key1_gpio,gpio_get_value(switch_data->key1_gpio));
	//pr_err("%s  gpio_get_value(%d)=%d\n",__func__,switch_data->key2_gpio,gpio_get_value(switch_data->key2_gpio));
	//pr_err("%s  gpio_get_value(%d)=%d\n",__func__,switch_data->key3_gpio,gpio_get_value(switch_data->key3_gpio));

      mutex_lock(&sem);
	key1=gpio_get_value(switch_data->key1_gpio);
	key2=gpio_get_value(switch_data->key2_gpio);
	key3=gpio_get_value(switch_data->key3_gpio);

	if(((key1==0)&&(key2==1)&&(key3==1))||((key1==1)&&(key2==0)&&(key3==1))||((key1==1)&&(key2==1)&&(key3==0)))
	{
		//gpio_set_value(switch_data->key3_gpio,0);
		//pr_err("%s  gpio_22get_value(%d)=%d\n",__func__,switch_data->key3_gpio,gpio_get_value(switch_data->key3_gpio));
		if(!key2)
		{
		    switch_set_state(&switch_data->sdev, MODE_DO_NOT_DISTURB);
		}

		if(!key3)
		{
		    switch_set_state(&switch_data->sdev, MODE_NORMAL);

		}

		if(!key1)
		{
		    switch_set_state(&switch_data->sdev, MODE_MUTE);
		}

        printk(KERN_ERR "%s ,tristatekey set to state(%d) \n",__func__,switch_data->sdev.state);
	}
	mutex_unlock(&sem);
}
irqreturn_t switch_dev_interrupt(int irq, void *_dev)
{
//printk("%s\n",__func__);
    schedule_work(&switch_data->work);

		return IRQ_HANDLED;
}

static void timer_handle(unsigned long arg)
{
    //mod_timer(&s_timer, jiffies + HZ);
  //  if(set_gpio_by_pinctrl() < 0)
   //      printk(KERN_ERR "tristate_key set_gpio_by_pinctrl FAILD!!!. \n");
    schedule_work(&switch_data->work);
    //del_timer(&switch_data->s_timer);

    //printk(KERN_ERR "tristate_key set gpio77 timer. \n");
}

/* //no need cause switch_class.c state_show()
static ssize_t switch_dev_print_state(struct switch_dev *sdev, char *buf)
{
	tri_mode_t state;
		state = switch_data->mode_type;

	if (state)
		return sprintf(buf, "%d\n", state);
	return -1;
}
*/

#ifdef CONFIG_OF
static int switch_dev_get_devtree_pdata(struct device *dev)
{
	struct device_node *node;

	node = dev->of_node;
	if (!node)
		return -EINVAL;

	switch_data->key3_gpio= of_get_named_gpio(node, "tristate,gpio_key3", 0);
	if ((!gpio_is_valid(switch_data->key3_gpio)))
		return -EINVAL;
	pr_err("switch_data->key3_gpio=%d \n", switch_data->key3_gpio);

	switch_data->key2_gpio= of_get_named_gpio(node, "tristate,gpio_key2", 0);
	if ((!gpio_is_valid(switch_data->key2_gpio)))
		return -EINVAL;
	pr_err("switch_data->key2_gpio=%d \n", switch_data->key2_gpio);
//printk("%s, key2 gpio:%d \n", __func__, switch_data->key2_gpio);

	switch_data->key1_gpio= of_get_named_gpio(node, "tristate,gpio_key1", 0);
	if ((!gpio_is_valid(switch_data->key1_gpio)))
		return -EINVAL;
	pr_err("switch_data->key1_gpio=%d \n", switch_data->key1_gpio);
//printk("%s, key1 gpio:%d \n", __func__, switch_data->key1_gpio);
	return 0;
}

static struct of_device_id tristate_dev_of_match[] = {
	{ .compatible = "oneplus,tri-state-key", },
	{ },
};
MODULE_DEVICE_TABLE(of, tristate_dev_of_match);

#else

static inline int
switch_dev_get_devtree_pdata(struct device *dev)
{
	return 0;
}
#endif
/*
#define SUPPLY_1V8		1800000UL
#define SUPPLY_IO_MIN		SUPPLY_1V8
#define SUPPLY_IO_MAX		SUPPLY_1V8
#define SUPPLY_IO_REQ_CURRENT	6000U

int tristate_regulator_release(void)
{



	if (switch_data->vdd_io != NULL) {
		regulator_put(switch_data->vdd_io);
		switch_data->vdd_io = NULL;
	}

	switch_data->power_enabled = false;

	return 0;
}

int tristate_regulator_set(bool enable)
{
	int error = 0;

	if (switch_data->vdd_io == NULL) {
		dev_err(switch_data->dev,
			"Regulators not set\n");
			return -EINVAL;
	}

	if (enable) {
		dev_dbg(switch_data->dev, "%s on\n", __func__);

		regulator_set_optimum_mode(switch_data->vdd_io,
					SUPPLY_IO_REQ_CURRENT);

		error = (regulator_is_enabled(switch_data->vdd_io) == 0) ?
					regulator_enable(switch_data->vdd_io) : 0;

		if (error) {
			dev_err(switch_data->dev,
				"Regulator vdd_io enable failed, error=%d\n",
				error);
			goto out_err;
		}

	} else {
		dev_dbg(switch_data->dev, "%s off\n", __func__);

		error = (switch_data->power_enabled &&
			regulator_is_enabled(switch_data->vdd_io) > 0) ?
				 regulator_disable(switch_data->vdd_io) : 0;

		if (error) {
			dev_err(switch_data->dev,
				"Regulator vdd_io disable failed, error=%d\n",
				 error);
			goto out_err;
		}

	}
    switch_data->power_enabled = enable;
	return 0;

out_err:
	tristate_regulator_release();
	return error;
}

static int tristate_supply_init(void)
{
	int error = 0;

	switch_data->vdd_io = regulator_get(switch_data->dev, "vdd_io");
	if (IS_ERR(switch_data->vdd_io)) {
		error = PTR_ERR(switch_data->vdd_io);
		dev_err(switch_data->dev,
			"Regulator get failed, vdd_io, error=%d\n", error);
		goto err;
	}

	if (regulator_count_voltages(switch_data->vdd_io) > 0) {
		error = regulator_set_voltage(switch_data->vdd_io,
						SUPPLY_IO_MIN, SUPPLY_IO_MAX);
		if (error) {
			dev_err(switch_data->dev,
				"regulator set(io) failed, error=%d\n", error);
			goto err;
		}
	}
	if (error) {
		dev_err(switch_data->dev,
			"regulator configuration failed.\n");
		goto err;
	}

	error = tristate_regulator_set(true);
	if (error) {
		dev_err(switch_data->dev,
			"regulator enable failed.\n");
		goto err;
	}

err:
	return error;
}

*/

static int tristate_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	int error=0;

	//void __iomem *cfg_reg;

        switch_data = kzalloc(sizeof(struct switch_dev_data), GFP_KERNEL);
        switch_data->dev = dev;
        #if 0
	    switch_data->key_pinctrl = devm_pinctrl_get(switch_data->dev);
         if (IS_ERR_OR_NULL(switch_data->key_pinctrl)) {
		        dev_err(switch_data->dev, "Failed to get pinctrl \n");
		        goto err_switch_dev_register;
	     }
         switch_data->set_state =pinctrl_lookup_state(switch_data->key_pinctrl,"pmx_tri_state_key_active");
         if (IS_ERR_OR_NULL(switch_data->set_state)) {
		        dev_err(switch_data->dev, "Failed to lookup_state \n");
		        goto err_switch_dev_register;
	     }

	     set_gpio_by_pinctrl();
		#endif
        //switch_data->last_type = MODE_UNKNOWN;

        //tristate_supply_init();
		error = switch_dev_get_devtree_pdata(dev);
		if (error) {
			dev_err(dev, "parse device tree fail!!!\n");
			goto err_switch_dev_register;
		}

		//config irq gpio and request irq
	switch_data->irq_key1 = gpio_to_irq(switch_data->key1_gpio);
       if (switch_data->irq_key1 <= 0)
       {
            printk("%s, irq number is not specified, irq #= %d, int pin=%d\n\n", __func__, switch_data->irq_key1, switch_data->key1_gpio);
            goto err_detect_irq_num_failed;
       }
       else
       {
        	error = gpio_request(switch_data->key1_gpio,"tristate_key1-int");
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, error);
        		goto err_request_gpio;
        	}
        	error = gpio_direction_input(switch_data->key1_gpio);
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, error);
        		goto err_set_gpio_input;
        	}

			error = request_irq(switch_data->irq_key1, switch_dev_interrupt,
			    IRQF_TRIGGER_FALLING, "tristate_key1", switch_data);

        	if (error) {
        		dev_err(dev,
        			"request_irq %i failed.\n",
        			switch_data->irq_key1);

        		switch_data->irq_key1 = -EINVAL;
        		goto err_request_irq;
            }
       }
       //config irq gpio and request irq
	 switch_data->irq_key2 = gpio_to_irq(switch_data->key2_gpio);
       if (switch_data->irq_key2 <= 0)
       {
            printk("%s, irq number is not specified, irq #= %d, int pin=%d\n\n", __func__, switch_data->irq_key2, switch_data->key2_gpio);
            goto err_detect_irq_num_failed;
       }
       else
       {
        	error = gpio_request(switch_data->key2_gpio,"tristate_key2-int");
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, error);
        		goto err_request_gpio;
        	}
        	error = gpio_direction_input(switch_data->key2_gpio);
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, error);
        		goto err_set_gpio_input;
        	}

			error = request_irq(switch_data->irq_key2, switch_dev_interrupt,
			    IRQF_TRIGGER_FALLING, "tristate_key2", switch_data);

        	if (error) {
        		dev_err(dev,
        			"request_irq %i failed.\n",
        			switch_data->irq_key2);

        		switch_data->irq_key2 = -EINVAL;
        		goto err_request_irq;
            }

       }

	   switch_data->irq_key3 = gpio_to_irq(switch_data->key3_gpio);
	   if (switch_data->irq_key3 <= 0)
	   {
	            printk("%s, irq number is not specified, irq #= %d, int pin=%d\n\n", __func__, \
	            switch_data->irq_key3, switch_data->key3_gpio);
	            goto err_detect_irq_num_failed;
       }
       else
       {
        	error = gpio_request(switch_data->key3_gpio,"tristate_key3-int");
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, error);
        		goto err_request_gpio;
        	}
        	error = gpio_direction_input(switch_data->key3_gpio);
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, error);
        		goto err_set_gpio_input;
        	}


			error = request_irq(switch_data->irq_key3, switch_dev_interrupt,
			    IRQF_TRIGGER_FALLING, "tristate_key3", switch_data);

        	if (error) {
        		dev_err(dev,
        			"request_irq %i failed.\n",
        			switch_data->irq_key3);

        		switch_data->irq_key3 = -EINVAL;
        		goto err_request_irq;
            }

       }


        INIT_WORK(&switch_data->work, switch_dev_work);

        init_timer(&switch_data->s_timer);
        switch_data->s_timer.function = &timer_handle;
        switch_data->s_timer.expires = jiffies + 5*HZ;

        add_timer(&switch_data->s_timer);

        enable_irq_wake(switch_data->irq_key1);
        enable_irq_wake(switch_data->irq_key2);
	    enable_irq_wake(switch_data->irq_key3);


        switch_data->sdev.name = DRV_NAME;
       	error = switch_dev_register(&switch_data->sdev);
	    if (error < 0)
		    goto err_request_gpio;
		 //set_gpio_by_pinctrl();
        //report the first switch
        //switch_dev_work(&switch_data->work);
        return 0;


err_request_gpio:
	switch_dev_unregister(&switch_data->sdev);
err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(switch_data->key2_gpio);
	gpio_free(switch_data->key1_gpio);
	gpio_free(switch_data->key3_gpio);
err_switch_dev_register:
	kfree(switch_data);

	return error;
}

static int tristate_dev_remove(struct platform_device *pdev)
{
printk("%s\n",__func__);
	cancel_work_sync(&switch_data->work);
	gpio_free(switch_data->key1_gpio);
	gpio_free(switch_data->key2_gpio);
	gpio_free(switch_data->key3_gpio);
	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

static struct platform_driver tristate_dev_driver = {
	.probe	= tristate_dev_probe,
	.remove	= tristate_dev_remove,
	.driver	= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tristate_dev_of_match),
	},
};
module_platform_driver(tristate_dev_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("switch Profiles by this triple key driver");

>>>>>>> 14eb53941c5374e2300b514b3a860507607404a0

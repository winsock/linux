/*
 * Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <soc/tegra/fuse.h>
#include <soc/tegra/xusb.h>

#include "xusb.h"

static inline struct tegra_xusb_padctl *
mbox_work_to_padctl(struct work_struct *work)
{
	return container_of(work, struct tegra_xusb_padctl, mbox_req_work);
}

static void tegra_xusb_phy_mbox_work(struct work_struct *work)
{
	struct tegra_xusb_padctl *padctl = mbox_work_to_padctl(work);
	struct tegra_xusb_mbox_msg *msg = &padctl->mbox_req;

	padctl->soc->handle_message(padctl, msg);
}

static void tegra_xusb_phy_mbox_rx(struct mbox_client *cl, void *data)
{
	struct tegra_xusb_padctl *padctl = dev_get_drvdata(cl->dev);
	struct tegra_xusb_mbox_msg *msg = data;

	if (padctl->soc->is_phy_message(msg)) {
		padctl->mbox_req = *msg;
		schedule_work(&padctl->mbox_req_work);
	}
}

static struct phy *tegra_xusb_pad_of_xlate(struct device *dev,
					   struct of_phandle_args *args)
{
	struct tegra_xusb_pad *pad = dev_get_drvdata(dev);
	struct phy *phy = NULL;
	unsigned int i;

	if (args->args_count != 0)
		return ERR_PTR(-EINVAL);

	for (i = 0; i < pad->soc->num_lanes; i++) {
		if (!pad->lanes[i])
			continue;

		if (pad->lanes[i]->dev.of_node == args->np) {
			phy = pad->lanes[i];
			break;
		}
	}

	if (phy == NULL)
		phy = ERR_PTR(-ENODEV);

	return phy;
}

static const struct of_device_id tegra_xusb_padctl_of_match[] = {
#if defined(CONFIG_ARCH_TEGRA_124_SOC) || defined(CONFIG_ARCH_TEGRA_132_SOC)
	{
		.compatible = "nvidia,tegra124-xusb-padctl",
		.data = &tegra124_xusb_padctl_soc,
	},
#endif
#if defined(CONFIG_ARCH_TEGRA_210_SOC)
	{
		.compatible = "nvidia,tegra210-xusb-padctl",
		.data = &tegra210_xusb_padctl_soc,
	},
#endif
	{ }
};
MODULE_DEVICE_TABLE(of, tegra_xusb_padctl_of_match);

static struct device_node *
tegra_xusb_find_pad_node(struct tegra_xusb_padctl *padctl, const char *name)
{
	/*
	 * of_find_node_by_name() drops a reference, so make sure to grab one.
	 */
	struct device_node *np = of_node_get(padctl->dev->of_node);

	np = of_find_node_by_name(np, "pads");
	if (np)
		np = of_find_node_by_name(np, name);

	return np;
}

static struct device_node *
tegra_xusb_pad_find_phy_node(struct tegra_xusb_pad *pad, unsigned int index)
{
	/*
	 * of_find_node_by_name() drops a reference, so make sure to grab one.
	 */
	struct device_node *np = of_node_get(pad->dev.of_node);
	char *name;

	name = kasprintf(GFP_KERNEL, "%s-%u", np->name, index);
	if (!name) {
		of_node_put(np);
		return NULL;
	}

	np = of_find_node_by_name(np, name);

	kfree(name);

	return np;
}

int tegra_xusb_lane_lookup_function(struct tegra_xusb_lane *lane,
				    const char *function)
{
	unsigned int i;

	for (i = 0; i < lane->soc->num_funcs; i++)
		if (strcmp(function, lane->soc->funcs[i]) == 0)
			return i;

	return -EINVAL;
}

int tegra_xusb_lane_parse_dt(struct tegra_xusb_lane *lane,
			     struct device_node *np)
{
	struct device *dev = &lane->pad->dev;
	const char *function;
	int err;

	err = of_property_read_string(np, "nvidia,function", &function);
	if (err < 0)
		return err;

	err = tegra_xusb_lane_lookup_function(lane, function);
	if (err < 0) {
		dev_err(dev, "invalid function \"%s\" for lane \"%s\"\n",
			function, np->name);
		return err;
	}

	lane->function = err;

	return 0;
}

static void tegra_xusb_lane_destroy(struct phy *phy)
{
	if (phy) {
		struct tegra_xusb_lane *lane = phy_get_drvdata(phy);

		lane->pad->ops->remove(lane);
		phy_destroy(phy);
	}
}

static void tegra_xusb_pad_release(struct device *dev)
{
	struct tegra_xusb_pad *pad = to_tegra_xusb_pad(dev);

	pad->soc->ops->remove(pad);
}

static struct device_type tegra_xusb_pad_type = {
	.release = tegra_xusb_pad_release,
};

int tegra_xusb_pad_init(struct tegra_xusb_pad *pad,
			struct tegra_xusb_padctl *padctl,
			struct device_node *np)
{
	int err;

	device_initialize(&pad->dev);
	INIT_LIST_HEAD(&pad->list);
	pad->dev.parent = padctl->dev;
	pad->dev.type = &tegra_xusb_pad_type;
	pad->dev.of_node = np;
	pad->padctl = padctl;

	err = dev_set_name(&pad->dev, "%s", pad->soc->name);
	if (err < 0)
		goto unregister;

	err = device_add(&pad->dev);
	if (err < 0)
		goto unregister;

	return 0;

unregister:
	device_unregister(&pad->dev);
	return err;
}

int tegra_xusb_pad_register(struct tegra_xusb_pad *pad,
			    const struct phy_ops *ops)
{
	struct phy *lane;
	unsigned int i;
	int err;

	pad->lanes = devm_kcalloc(&pad->dev, pad->soc->num_lanes, sizeof(lane),
				  GFP_KERNEL);
	if (!pad->lanes)
		return -ENOMEM;

	for (i = 0; i < pad->soc->num_lanes; i++) {
		struct device_node *np = tegra_xusb_pad_find_phy_node(pad, i);
		struct tegra_xusb_lane *lane;

		/* skip disabled lanes */
		if (!np || !of_device_is_available(np))
			continue;

		pad->lanes[i] = phy_create(&pad->dev, np, ops);
		if (IS_ERR(pad->lanes[i])) {
			err = PTR_ERR(pad->lanes[i]);
			goto remove;
		}

		lane = pad->ops->probe(pad, np, i);
		if (IS_ERR(lane)) {
			phy_destroy(pad->lanes[i]);
			err = PTR_ERR(lane);
			goto remove;
		}

		list_add_tail(&lane->list, &pad->padctl->lanes);
		phy_set_drvdata(pad->lanes[i], lane);
	}

	pad->provider = of_phy_provider_register(&pad->dev,
						 tegra_xusb_pad_of_xlate);
	if (IS_ERR(pad->provider)) {
		err = PTR_ERR(pad->provider);
		goto remove;
	}

	return 0;

remove:
	while (i--)
		tegra_xusb_lane_destroy(pad->lanes[i]);

	return err;
}

void tegra_xusb_pad_unregister(struct tegra_xusb_pad *pad)
{
	unsigned int i = pad->soc->num_lanes;

	of_phy_provider_unregister(pad->provider);

	while (i--)
		tegra_xusb_lane_destroy(pad->lanes[i]);

	device_unregister(&pad->dev);
}

static struct tegra_xusb_pad *
tegra_xusb_pad_create(struct tegra_xusb_padctl *padctl,
		      const struct tegra_xusb_pad_soc *soc)
{
	struct tegra_xusb_pad *pad;
	struct device_node *np;
	int err;

	np = tegra_xusb_find_pad_node(padctl, soc->name);
	if (!np || !of_device_is_available(np))
		return NULL;

	pad = soc->ops->probe(padctl, soc, np);
	if (IS_ERR(pad)) {
		err = PTR_ERR(pad);
		dev_err(padctl->dev, "failed to create pad %s: %d\n",
			soc->name, err);
		return ERR_PTR(err);
	}

	/* XXX move this into ->probe() to avoid string comparison */
	if (strcmp(soc->name, "pcie") == 0)
		padctl->pcie = pad;

	if (strcmp(soc->name, "sata") == 0)
		padctl->sata = pad;

	if (strcmp(soc->name, "utmi") == 0)
		padctl->utmi = pad;

	if (strcmp(soc->name, "ulpi") == 0)
		padctl->ulpi = pad;

	if (strcmp(soc->name, "hsic") == 0)
		padctl->hsic = pad;

	return pad;
}

static void __tegra_xusb_remove_pads(struct tegra_xusb_padctl *padctl)
{
	struct tegra_xusb_pad *pad, *tmp;

	list_for_each_entry_safe_reverse(pad, tmp, &padctl->pads, list) {
		list_del(&pad->list);
		tegra_xusb_pad_unregister(pad);
	}
}

static void tegra_xusb_remove_pads(struct tegra_xusb_padctl *padctl)
{
	mutex_lock(&padctl->lock);
	__tegra_xusb_remove_pads(padctl);
	mutex_unlock(&padctl->lock);
}

static void tegra_xusb_lane_program(struct tegra_xusb_lane *lane)
{
	struct tegra_xusb_padctl *padctl = lane->pad->padctl;
	const struct tegra_xusb_lane_soc *soc = lane->soc;
	u32 value;

	/* choose function */
	value = padctl_readl(padctl, soc->offset);
	value &= ~(soc->mask << soc->shift);
	value |= lane->function << soc->shift;
	padctl_writel(padctl, value, soc->offset);
}

static void tegra_xusb_pad_program(struct tegra_xusb_pad *pad)
{
	unsigned int i;

	for (i = 0; i < pad->soc->num_lanes; i++) {
		struct tegra_xusb_lane *lane;

		if (pad->lanes[i]) {
			lane = phy_get_drvdata(pad->lanes[i]);
			tegra_xusb_lane_program(lane);
		}
	}
}

static int tegra_xusb_setup_pads(struct tegra_xusb_padctl *padctl)
{
	struct tegra_xusb_pad *pad;
	unsigned int i;

	mutex_lock(&padctl->lock);

	for (i = 0; i < padctl->soc->num_pads; i++) {
		const struct tegra_xusb_pad_soc *soc = padctl->soc->pads[i];
		int err;

		pad = tegra_xusb_pad_create(padctl, soc);
		if (IS_ERR(pad)) {
			err = PTR_ERR(pad);
			dev_err(padctl->dev, "failed to create pad %s: %d\n",
				soc->name, err);
			__tegra_xusb_remove_pads(padctl);
			mutex_unlock(&padctl->lock);
			return err;
		}

		if (!pad)
			continue;

		list_add_tail(&pad->list, &padctl->pads);
	}

	list_for_each_entry(pad, &padctl->pads, list)
		tegra_xusb_pad_program(pad);

	mutex_unlock(&padctl->lock);
	return 0;
}

static bool tegra_xusb_lane_check(struct tegra_xusb_lane *lane,
				  const char *function)
{
	if (lane) {
		const char *func = lane->soc->funcs[lane->function];

		return strcmp(function, func) == 0;
	}

	return false;
}

struct tegra_xusb_lane *tegra_xusb_find_lane(struct tegra_xusb_padctl *padctl,
					     const char *type,
					     unsigned int index)
{
	struct tegra_xusb_lane *lane, *hit = NULL;
	char *name;

	name = kasprintf(GFP_KERNEL, "%s-%u", type, index);
	if (!name)
		return NULL;

	list_for_each_entry(lane, &padctl->lanes, list) {
		if (strcmp(lane->soc->name, name) == 0) {
			hit = lane;
			break;
		}
	}

	kfree(name);
	return hit;
}

struct tegra_xusb_lane *
tegra_xusb_port_find_lane(struct tegra_xusb_port *port,
			  const struct tegra_xusb_lane_map *map,
			  const char *function)
{
	struct tegra_xusb_lane *lane, *match = NULL;

	for (map = map; map->type; map++) {
		if (port->index != map->port)
			continue;

		lane = tegra_xusb_find_lane(port->padctl, map->type,
					    map->index);
		if (!tegra_xusb_lane_check(lane, function))
			continue;

		if (match)
			dev_err(&port->dev, "conflicting match: %s-%u / %s\n",
				map->type, map->index, match->soc->name);
		else
			match = lane;
	}

	return match;
}

static struct device_node *
tegra_xusb_find_port_node(struct tegra_xusb_padctl *padctl, const char *type,
			  unsigned int index)
{
	/*
	 * of_find_node_by_name() drops a reference, so make sure to grab one.
	 */
	struct device_node *np = of_node_get(padctl->dev->of_node);

	np = of_find_node_by_name(np, "ports");
	if (np) {
		char *name;

		name = kasprintf(GFP_KERNEL, "%s-%u", type, index);
		np = of_find_node_by_name(np, name);
		kfree(name);
	}

	return np;
}

struct tegra_xusb_port *
tegra_xusb_find_port(struct tegra_xusb_padctl *padctl, const char *type,
		     unsigned int index)
{
	struct tegra_xusb_port *port;
	struct device_node *np;

	np = tegra_xusb_find_port_node(padctl, type, index);
	if (!np)
		return NULL;

	list_for_each_entry(port, &padctl->ports, list) {
		if (np == port->dev.of_node) {
			of_node_put(np);
			return port;
		}
	}

	of_node_put(np);

	return NULL;
}

struct tegra_xusb_utmi_port *
tegra_xusb_find_utmi_port(struct tegra_xusb_padctl *padctl, unsigned int index)
{
	struct tegra_xusb_port *port;

	port = tegra_xusb_find_port(padctl, "utmi", index);
	if (port)
		return to_utmi_port(port);

	return NULL;
}

struct tegra_xusb_usb3_port *
tegra_xusb_find_usb3_port(struct tegra_xusb_padctl *padctl, unsigned int index)
{
	struct tegra_xusb_port *port;

	port = tegra_xusb_find_port(padctl, "usb3", index);
	if (port)
		return to_usb3_port(port);

	return NULL;
}

static int tegra_xusb_port_init(struct tegra_xusb_port *port,
				struct tegra_xusb_padctl *padctl,
				struct device_node *np,
				const char *name,
				unsigned int index)
{
	int err;

	INIT_LIST_HEAD(&port->list);
	port->padctl = padctl;
	port->index = index;

	device_initialize(&port->dev);
	port->dev.of_node = of_node_get(np);
	port->dev.parent = padctl->dev;

	err = dev_set_name(&port->dev, "%s-%u", name, index);
	if (err < 0)
		goto unregister;

	err = device_add(&port->dev);
	if (err < 0)
		goto unregister;

	return 0;

unregister:
	device_unregister(&port->dev);
	return err;
}

static void tegra_xusb_port_unregister(struct tegra_xusb_port *port)
{
	device_unregister(&port->dev);
}

static int tegra_xusb_utmi_port_parse_dt(struct tegra_xusb_utmi_port *utmi)
{
	struct tegra_xusb_port *port = &utmi->base;
	struct device_node *np = port->dev.of_node;

	utmi->internal = of_property_read_bool(np, "nvidia,internal");

	utmi->supply = devm_regulator_get(&port->dev, "vbus");
	if (IS_ERR(utmi->supply))
		return PTR_ERR(utmi->supply);

	return 0;
}

static int tegra_xusb_add_utmi_port(struct tegra_xusb_padctl *padctl,
				    unsigned int index)
{
	struct tegra_xusb_utmi_port *utmi;
	struct device_node *np;
	int err = 0;

	/*
	 * UTMI ports don't require additional properties, but if the port is
	 * marked as disabled there is no reason to register it.
	 */
	np = tegra_xusb_find_port_node(padctl, "utmi", index);
	if (!np || !of_device_is_available(np))
		goto out;

	utmi = devm_kzalloc(padctl->dev, sizeof(*utmi), GFP_KERNEL);
	if (!utmi) {
		err = -ENOMEM;
		goto out;
	}

	err = tegra_xusb_port_init(&utmi->base, padctl, np, "utmi", index);
	if (err < 0)
		goto out;

	utmi->base.ops = padctl->soc->ports.utmi.ops;

	utmi->base.lane = utmi->base.ops->map(&utmi->base);
	if (IS_ERR(utmi->base.lane)) {
		err = PTR_ERR(utmi->base.lane);
		goto out;
	}

	err = tegra_xusb_utmi_port_parse_dt(utmi);
	if (err < 0) {
		tegra_xusb_port_unregister(&utmi->base);
		goto out;
	}

	list_add_tail(&utmi->base.list, &padctl->ports);

out:
	of_node_put(np);
	return err;
}

static int tegra_xusb_ulpi_port_parse_dt(struct tegra_xusb_ulpi_port *ulpi)
{
	struct tegra_xusb_port *port = &ulpi->base;
	struct device_node *np = port->dev.of_node;

	ulpi->internal = of_property_read_bool(np, "nvidia,internal");

	return 0;
}

static int tegra_xusb_add_ulpi_port(struct tegra_xusb_padctl *padctl,
				    unsigned int index)
{
	struct tegra_xusb_ulpi_port *ulpi;
	struct device_node *np;
	int err = 0;

	np = tegra_xusb_find_port_node(padctl, "ulpi", index);
	if (!np || !of_device_is_available(np))
		goto out;

	ulpi = devm_kzalloc(padctl->dev, sizeof(*ulpi), GFP_KERNEL);
	if (!ulpi) {
		err = -ENOMEM;
		goto out;
	}

	err = tegra_xusb_port_init(&ulpi->base, padctl, np, "ulpi", index);
	if (err < 0)
		goto out;

	ulpi->base.ops = padctl->soc->ports.ulpi.ops;

	ulpi->base.lane = ulpi->base.ops->map(&ulpi->base);
	if (IS_ERR(ulpi->base.lane)) {
		err = PTR_ERR(ulpi->base.lane);
		goto out;
	}

	err = tegra_xusb_ulpi_port_parse_dt(ulpi);
	if (err < 0) {
		tegra_xusb_port_unregister(&ulpi->base);
		goto out;
	}

	list_add_tail(&ulpi->base.list, &padctl->ports);

out:
	of_node_put(np);
	return err;
}

static int tegra_xusb_hsic_port_parse_dt(struct tegra_xusb_hsic_port *hsic)
{
	/* XXX */
	return 0;
}

static int tegra_xusb_add_hsic_port(struct tegra_xusb_padctl *padctl,
				    unsigned int index)
{
	struct tegra_xusb_hsic_port *hsic;
	struct device_node *np;
	int err = 0;

	np = tegra_xusb_find_port_node(padctl, "hsic", index);
	if (!np || !of_device_is_available(np))
		goto out;

	hsic = devm_kzalloc(padctl->dev, sizeof(*hsic), GFP_KERNEL);
	if (!hsic) {
		err = -ENOMEM;
		goto out;
	}

	err = tegra_xusb_port_init(&hsic->base, padctl, np, "hsic", index);
	if (err < 0)
		goto out;

	hsic->base.ops = padctl->soc->ports.hsic.ops;

	hsic->base.lane = hsic->base.ops->map(&hsic->base);
	if (IS_ERR(hsic->base.lane)) {
		err = PTR_ERR(hsic->base.lane);
		goto out;
	}

	err = tegra_xusb_hsic_port_parse_dt(hsic);
	if (err < 0) {
		tegra_xusb_port_unregister(&hsic->base);
		goto out;
	}

	list_add_tail(&hsic->base.list, &padctl->ports);

out:
	of_node_put(np);
	return err;
}

static int tegra_xusb_usb3_port_parse_dt(struct tegra_xusb_usb3_port *usb3)
{
	struct tegra_xusb_port *port = &usb3->base;
	struct device_node *np = port->dev.of_node;
	u32 value;
	int err;

	err = of_property_read_u32_index(np, "nvidia,port", 0, &value);
	if (err < 0) {
		dev_err(&port->dev, "failed to read port: %d\n", err);
		return err;
	}

	usb3->port = value;

	usb3->internal = of_property_read_bool(np, "nvidia,internal");

	usb3->supply = devm_regulator_get(&port->dev, "vbus");
	if (IS_ERR(usb3->supply))
		return PTR_ERR(usb3->supply);

	return 0;
}

static int tegra_xusb_add_usb3_port(struct tegra_xusb_padctl *padctl,
				    unsigned int index)
{
	struct tegra_xusb_usb3_port *usb3;
	struct device_node *np;
	int err = 0;

	/*
	 * If there is no supplemental configuration in the device tree the
	 * port is unusable. But it is valid to configure only a single port,
	 * hence return 0 instead of an error to allow ports to be optional.
	 */
	np = tegra_xusb_find_port_node(padctl, "usb3", index);
	if (!np || !of_device_is_available(np))
		goto out;

	usb3 = devm_kzalloc(padctl->dev, sizeof(*usb3), GFP_KERNEL);
	if (!usb3) {
		err = -ENOMEM;
		goto out;
	}

	err = tegra_xusb_port_init(&usb3->base, padctl, np, "usb3", index);
	if (err < 0)
		goto out;

	usb3->base.ops = padctl->soc->ports.usb3.ops;

	usb3->base.lane = usb3->base.ops->map(&usb3->base);
	if (IS_ERR(usb3->base.lane)) {
		err = PTR_ERR(usb3->base.lane);
		goto out;
	}

	err = tegra_xusb_usb3_port_parse_dt(usb3);
	if (err < 0) {
		tegra_xusb_port_unregister(&usb3->base);
		goto out;
	}

	list_add_tail(&usb3->base.list, &padctl->ports);

out:
	of_node_put(np);
	return err;
}

static int tegra_xusb_setup_ports(struct tegra_xusb_padctl *padctl)
{
	struct tegra_xusb_port *port;
	unsigned int i;
	int err = 0;

	mutex_lock(&padctl->lock);

	for (i = 0; i < padctl->soc->ports.utmi.count; i++) {
		err = tegra_xusb_add_utmi_port(padctl, i);
		if (err < 0)
			goto unlock;
	}

	for (i = 0; i < padctl->soc->ports.ulpi.count; i++) {
		err = tegra_xusb_add_ulpi_port(padctl, i);
		if (err < 0)
			goto unlock;
	}

	for (i = 0; i < padctl->soc->ports.hsic.count; i++) {
		err = tegra_xusb_add_hsic_port(padctl, i);
		if (err < 0)
			goto unlock;
	}

	for (i = 0; i < padctl->soc->ports.usb3.count; i++) {
		err = tegra_xusb_add_usb3_port(padctl, i);
		if (err < 0)
			goto unlock;
	}

	list_for_each_entry(port, &padctl->ports, list) {
		err = port->ops->enable(port);
		if (err < 0)
			dev_err(padctl->dev, "failed to enable port %s: %d\n",
				dev_name(&port->dev), err);
	}

unlock:
	mutex_unlock(&padctl->lock);
	return err;
}

static void tegra_xusb_remove_ports(struct tegra_xusb_padctl *padctl)
{
	struct tegra_xusb_port *port, *tmp;

	mutex_lock(&padctl->lock);

	list_for_each_entry_safe_reverse(port, tmp, &padctl->ports, list) {
		list_del(&port->list);
		tegra_xusb_port_unregister(port);
	}

	mutex_unlock(&padctl->lock);
}

static int tegra_xusb_padctl_probe(struct platform_device *pdev)
{
	struct device_node *np = of_node_get(pdev->dev.of_node);
	const struct tegra_xusb_padctl_soc *soc;
	struct tegra_xusb_padctl *padctl;
	const struct of_device_id *match;
	struct resource *res;
	int err;

	/* for backwards compatibility with old device trees */
	np = of_find_node_by_name(np, "pads");
	if (!np) {
		dev_warn(&pdev->dev, "deprecated DT, using legacy driver\n");
		return tegra_xusb_padctl_legacy_probe(pdev);
	}

	of_node_put(np);

	match = of_match_node(tegra_xusb_padctl_of_match, pdev->dev.of_node);
	soc = match->data;

	padctl = soc->ops->probe(&pdev->dev, soc);
	if (IS_ERR(padctl))
		return PTR_ERR(padctl);

	platform_set_drvdata(pdev, padctl);
	INIT_LIST_HEAD(&padctl->ports);
	INIT_LIST_HEAD(&padctl->lanes);
	INIT_LIST_HEAD(&padctl->pads);
	mutex_init(&padctl->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	padctl->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(padctl->regs)) {
		err = PTR_ERR(padctl->regs);
		goto remove;
	}

	padctl->rst = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(padctl->rst)) {
		err = PTR_ERR(padctl->rst);
		goto remove;
	}

	err = reset_control_deassert(padctl->rst);
	if (err < 0)
		goto remove;

	err = tegra_xusb_setup_pads(padctl);
	if (err < 0) {
		dev_err(&pdev->dev, "failed to setup pads: %d\n", err);
		goto reset;
	}

	INIT_WORK(&padctl->mbox_req_work, tegra_xusb_phy_mbox_work);
	padctl->mbox_client.dev = &pdev->dev;
	padctl->mbox_client.tx_block = true;
	padctl->mbox_client.tx_tout = 0;
	padctl->mbox_client.rx_callback = tegra_xusb_phy_mbox_rx;

	padctl->mbox_chan = mbox_request_channel(&padctl->mbox_client, 0);
	if (IS_ERR(padctl->mbox_chan)) {
		err = PTR_ERR(padctl->mbox_chan);
		if (err == -EPROBE_DEFER) {
			dev_dbg(&pdev->dev, "mailbox not available yet\n");
			goto remove_pads;
		} else {
			dev_warn(&pdev->dev,
				 "failed to get mailbox, USB support disabled");
		}
	} else {
		err = tegra_xusb_setup_ports(padctl);
		if (err)
			goto remove_pads;
	}

	return 0;

remove_pads:
	tegra_xusb_remove_pads(padctl);
reset:
	reset_control_assert(padctl->rst);
remove:
	soc->ops->remove(padctl);
	return err;
}

static int tegra_xusb_padctl_remove(struct platform_device *pdev)
{
	struct tegra_xusb_padctl *padctl = platform_get_drvdata(pdev);
	int err;

	if (!IS_ERR(padctl->mbox_chan)) {
		cancel_work_sync(&padctl->mbox_req_work);
		mbox_free_channel(padctl->mbox_chan);
	}

	tegra_xusb_remove_ports(padctl);
	tegra_xusb_remove_pads(padctl);

	err = reset_control_assert(padctl->rst);
	if (err < 0)
		dev_err(&pdev->dev, "failed to assert reset: %d\n", err);

	padctl->soc->ops->remove(padctl);

	return err;
}

static struct platform_driver tegra_xusb_padctl_driver = {
	.driver = {
		.name = "tegra-xusb-padctl",
		.of_match_table = tegra_xusb_padctl_of_match,
	},
	.probe = tegra_xusb_padctl_probe,
	.remove = tegra_xusb_padctl_remove,
};
module_platform_driver(tegra_xusb_padctl_driver);

MODULE_AUTHOR("Thierry Reding <treding@nvidia.com>");
MODULE_DESCRIPTION("Tegra XUSB Pad Controller driver");
MODULE_LICENSE("GPL v2");

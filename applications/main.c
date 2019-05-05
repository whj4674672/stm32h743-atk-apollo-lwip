/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-05     whj4674672   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "netif_port.h"
#include "main.h"
/* defined the LED0 pin: PB1 */
#define LED0_PIN    GET_PIN(B, 1)
struct netif gnetif; /* network interface structure */

/**
  * @brief  Initializes the lwIP stack
  * @param  None
  * @retval None
  */
static void Netif_Config(void)
{
    ip_addr_t ipaddr;
    ip_addr_t netmask;
    ip_addr_t gw;

    IP_ADDR4(&ipaddr,192,168,1,30);
    IP_ADDR4(&netmask,255,255,255,0);
    IP_ADDR4(&gw,192,168,1,1);

    /* add the network interface */
    netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

    /*  Registers the default network interface. */
    netif_set_default(&gnetif);

    netif_set_link_callback(&gnetif, eth_link_callback);

    rt_thread_t tid = rt_thread_create("eth_link", ethernet_link_thread, &gnetif, 1024, 20, 10);
    rt_thread_startup(tid);


#if LWIP_DHCP
    /* Start DHCPClient */
    tid = rt_thread_create("eth_dhcp", DHCP_Thread, &gnetif, 1024, 30, 20);
    rt_thread_startup(tid);
#endif
}


int main(void)
{
    int count = 1;
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    tcpip_init(NULL, NULL);
    Netif_Config();

    while (count++)
    {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(500);
    }
    return RT_EOK;
}

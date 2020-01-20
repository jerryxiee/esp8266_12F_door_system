/* * ESPRESSIF MIT License
 *
 * Copyright (c) 2016 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

////////////////////////////////////////////
//
// 工程：	SQ_Door
//
// 平台：	【技新电子】物联网开发板 ESP8266 V1.0
//
// 功能：	①：8266设置为STA模式，接入WIFI热点
//
//			②：如果WIFI连接出错，进入【微信配网】
//
//			②：设置TCP服务
//
//			④：设置继电器

////////////////////////////////////////////


// 头文件引用
//==================================================================================
#include "user_config.h"		// 用户配置
#include "driver/uart.h"  		// 串口
#include "driver/oled.h"  		// OLED

//#include "at_custom.h"
#include "c_types.h"			// 变量类型
#include "eagle_soc.h"			// GPIO函数、宏定义
#include "ip_addr.h"			// 被"espconn.h"使用。
#include "espconn.h"			// TCP/UDP接口
//#include "espnow.h"
#include "ets_sys.h"			// 回调函数
//#include "gpio.h"
#include "mem.h"				// 内存申请等函数
#include "os_type.h"			// os_XXX
#include "osapi.h"  			// os_XXX、软件定时器
//#include "ping.h"
//#include "pwm.h"
//#include "queue.h"
#include "smartconfig.h"		// 智能配网
//#include "spi_flash.h"
//#include "upgrade.h"
#include "user_interface.h" 	// 系统接口、system_param_xxx接口、WIFI、RateContro
//==================================================================================


// 宏定义
//==================================================================================
#define		ProjectName			"SQ_Door_System"		// 工程名宏定义

#define		Sector_STA_INFO		0x90			// 【STA参数】保存扇区

#define		LED_ON				GPIO_OUTPUT_SET(GPIO_ID_PIN(4),0)		// LED亮
#define		LED_OFF				GPIO_OUTPUT_SET(GPIO_ID_PIN(4),1)		// LED灭

#define		JDQ_ON				GPIO_DIS_OUTPUT(GPIO_ID_PIN(12));		// 开门
#define		JDQ_OFF				GPIO_OUTPUT_SET(GPIO_ID_PIN(12),1);		// 锁门
//==================================================================================

// 全局变量
//==================================================================================
struct station_config STA_INFO;		// 【STA】参数结构体

u8 C_LED_Flash = 0;					// LED闪烁计次

os_timer_t OS_Timer_IP;				// 定时器_查询IP地址

os_timer_t OS_Timer_1;			// 定义软件定时器

struct espconn ST_NetCon;		// 网络连接结构体

// 注：OS_Timer_1必须定义为全局变量，因为ESP8266的内核还要使用
//--------------------------------------------------------------------
os_timer_t OS_Timer_1;	// ①：定义软件定时器(os_timer_t型结构体)
//==================================================================================

// 毫秒延时函数
//===========================================
void ICACHE_FLASH_ATTR delay_ms(u32 C_time)
{	for(;C_time>0;C_time--)
		os_delay_us(1000);
}
//===========================================

// 软件定时的回调函数
//======================================================================
void ICACHE_FLASH_ATTR OS_Timer_1_cb(void)		// ②：定义回调函数
{
	LED_OFF; // 首字母为'g'/'G'，灯灭
	JDQ_OFF;
	os_timer_disarm(&OS_Timer_1);
	os_printf("\r\n----OS_Timer_1_cb----\r\n");	// 进入回调函数标志
}
//======================================================================


// 软件定时器初始化(ms毫秒)
//================================================================================================
void ICACHE_FLASH_ATTR OS_Timer_1_Init_JX(u32 time_ms, u8 time_repetitive)
{
	// 关闭定时器
	// 参数一：要关闭的定时器
	//--------------------------------------------------------
	os_timer_disarm(&OS_Timer_1);	// ③：关闭软件定时器


	// 设置定时器
	// 参数一：要设置的定时器；参数二：回调函数(需类型转换)；参数三：回调函数的参数
	//【注：os_timer_setfn必须在软件定时器未使能的情况下调用】
	//------------------------------------------------------------------------------------------
	os_timer_setfn(&OS_Timer_1,(os_timer_func_t *)OS_Timer_1_cb, NULL);	// ④：设置回调函数


	// 使能(启动)ms定时器
	// 参数一：要使能的定时器；参数二：定时时间（单位：ms）；参数三：1=重复/0=只一次
	//------------------------------------------------------------------------------------------
	os_timer_arm(&OS_Timer_1, time_ms, time_repetitive);  // ⑤：设置定时器参数并使能定时器
	//-------------------------------------------------------------------
	// 【如未调用system_timer_reinit，可支持范围：[5ms ～ 6,870,947ms]】
	// 【如果调用system_timer_reinit，可支持范围：[100ms ～ 428,496 ms]】
	//-------------------------------------------------------------------
}
//================================================================================================


// SmartConfig状态发生改变时，进入此回调函数
//--------------------------------------------
// 参数1：sc_status status / 参数2：无类型指针【在不同状态下，[void *pdata]的传入参数是不同的】
//=================================================================================================================
void ICACHE_FLASH_ATTR smartconfig_done(sc_status status, void *pdata)
{
	os_printf("\r\n------ smartconfig_done ------\r\n");	// ESP8266网络状态改变

    switch(status)
    {
    	// CmartConfig等待
		//……………………………………………………
		case SC_STATUS_WAIT:		// 初始值
			os_printf("\r\nSC_STATUS_WAIT\r\n");
		break;
		//……………………………………………………

		// 发现【WIFI信号】（8266在这种状态下等待配网）
		//…………………………………………………………………………………………………
		case SC_STATUS_FIND_CHANNEL:
			os_printf("\r\nSC_STATUS_FIND_CHANNEL\r\n");

			os_printf("\r\n---- Please Use WeChat to SmartConfig ------\r\n\r\n");

			OLED_ShowString(0,0,"ESP8266 = STA");
			OLED_ShowString(0,2,"Use WeChat to   ");
			OLED_ShowString(0,4,"SmartConfig     ");
			OLED_ShowString(0,6,"................");
		break;
		//…………………………………………………………………………………………………

        // 正在获取【SSID】【PSWD】
        //…………………………………………………………………………………………………
        case SC_STATUS_GETTING_SSID_PSWD:
            os_printf("\r\nSC_STATUS_GETTING_SSID_PSWD\r\n");

            // 【SC_STATUS_GETTING_SSID_PSWD】状态下，参数2==SmartConfig类型指针
            //-------------------------------------------------------------------
			sc_type *type = pdata;		// 获取【SmartConfig类型】指针

			// 配网方式 == 【ESPTOUCH】
			//-------------------------------------------------
            if (*type == SC_TYPE_ESPTOUCH)
            { os_printf("\r\nSC_TYPE:SC_TYPE_ESPTOUCH\r\n"); }

            // 配网方式 == 【AIRKISS】||【ESPTOUCH_AIRKISS】
            //-------------------------------------------------
            else
            { os_printf("\r\nSC_TYPE:SC_TYPE_AIRKISS\r\n"); }

	    break;
	    //…………………………………………………………………………………………………

	    // 成功获取到【SSID】【PSWD】，保存STA参数，并连接WIFI
	    //…………………………………………………………………………………………………
        case SC_STATUS_LINK:
            os_printf("\r\nSC_STATUS_LINK\r\n");

            // 【SC_STATUS_LINK】状态下，参数2 == STA参数指针
            //------------------------------------------------------------------
            struct station_config *sta_conf = pdata;	// 获取【STA参数】指针

            // 将【SSID】【PASS】保存到【外部Flash】中
            //--------------------------------------------------------------------------
			spi_flash_erase_sector(Sector_STA_INFO);						// 擦除扇区
			spi_flash_write(Sector_STA_INFO*4096, (uint32 *)sta_conf, 96);	// 写入扇区
			//--------------------------------------------------------------------------

	        wifi_station_set_config(sta_conf);			// 设置STA参数【Flash】
	        wifi_station_disconnect();					// 断开STA连接
	        wifi_station_connect();						// ESP8266连接WIFI

	        OLED_ShowString(0,0,"ESP8266 = STA");
	        OLED_ShowString(0,2,"WIFI Connecting");
	        OLED_ShowString(0,4,"................");
	        OLED_ShowString(0,6,"................");

	    break;
	    //…………………………………………………………………………………………………


        // ESP8266作为STA，成功连接到WIFI
	    //…………………………………………………………………………………………………
        case SC_STATUS_LINK_OVER:
            os_printf("\r\nSC_STATUS_LINK_OVER\r\n");

            smartconfig_stop();		// 停止SmartConfig

            //**************************************************************************************************
//            struct ip_info ST_ESP8266_IP;	// ESP8266的IP信息
//            u8 ESP8266_IP[4];				// ESP8266的IP地址
//			wifi_get_ip_info(STATION_IF,&ST_ESP8266_IP);	// 获取8266_STA的IP地址
//
//			ESP8266_IP[0] = ST_ESP8266_IP.ip.addr;		// IP地址高八位 == addr低八位
//			ESP8266_IP[1] = ST_ESP8266_IP.ip.addr>>8;	// IP地址次高八位 == addr次低八位
//			ESP8266_IP[2] = ST_ESP8266_IP.ip.addr>>16;	// IP地址次低八位 == addr次高八位
//			ESP8266_IP[3] = ST_ESP8266_IP.ip.addr>>24;	// IP地址低八位 == addr高八位

			// 显示ESP8266的IP地址
			//-----------------------------------------------------------------------------------------------
//			os_printf("ESP8266_IP = %d.%d.%d.%d\n",ESP8266_IP[0],ESP8266_IP[1],ESP8266_IP[2],ESP8266_IP[3]);
//			OLED_ShowIP(24,2,ESP8266_IP);	// OLED显示ESP8266的IP地址
//			OLED_ShowString(0,4,"Connect to WIFI ");
//			OLED_ShowString(0,6,"Successfully    ");
			//-----------------------------------------------------------------------------------------------

    		//----------------------------------------------------------------
    		OLED_ShowString(0,0,"ESP8266 = STA");
    		OLED_ShowString(0,2,"Success Connected Wifi");
    		OLED_ShowString(0,4,"................");
    		OLED_ShowString(0,6,"................");
    		//----------------------------------------------------------------

			// 接入WIFI成功后，LED快闪3次
			//----------------------------------------------------
			for(; C_LED_Flash<=5; C_LED_Flash++)
			{
				GPIO_OUTPUT_SET(GPIO_ID_PIN(4),(C_LED_Flash%2));
				delay_ms(100);
			}

			os_printf("\r\n---- ESP8266 Connect to WIFI Successfully ----\r\n");
			//**************************************************************************************************

	    break;
	    //…………………………………………………………………………………………………

    }
}
//=================================================================================================================

// 成功发送网络数据的回调函数
//==========================================================
void ICACHE_FLASH_ATTR ESP8266_WIFI_Send_Cb_JX(void *arg)
{
	os_printf("\nESP8266_WIFI_Send_OK\n");
}
//==========================================================

// 成功接收网络数据的回调函数【参数1：网络传输结构体espconn指针、参数2：网络传输数据指针、参数3：数据长度】
//=========================================================================================================
void ICACHE_FLASH_ATTR ESP8266_WIFI_Recv_Cb_JX(void * arg, char * pdata, unsigned short len)
{
	struct espconn * T_arg = arg;		// 缓存网络连接结构体指针

	// 根据数据设置LED的亮/灭
	//-------------------------------------------------------------------------------
	if(pdata[0] == 'k' || pdata[0] == 'K')	{
		LED_ON;// 首字母为'k'/'K'，灯亮

		JDQ_ON;

		//定时任务
		OS_Timer_1_Init_JX(2000,1);		// 500ms(重复)

		espconn_send(T_arg,"ESP8266_WIFI_Recv_OPEN",os_strlen("ESP8266_WIFI_Recv_OPEN"));
	}
	else if(pdata[0] == 'g' || pdata[0] == 'G')	{
		LED_OFF; // 首字母为'g'/'G'，灯灭

		LED_OFF;

		espconn_send(T_arg,"ESP8266_WIFI_Recv_CLOSE",os_strlen("ESP8266_WIFI_Recv_CLOSE"));
	}

	os_printf("\nESP8266_Receive_Data = %s\n",pdata);		// 串口打印接收到的数据

	/*
	// 获取远端信息
	//------------------------------------------------------------------------------------
	remot_info * P_port_info = NULL;	// 定义远端连接信息指针
	if(espconn_get_connection_info(T_arg, &P_port_info, 0)==ESPCONN_OK)	// 获取远端信息
	{
		T_arg->proto.tcp->remote_port  = P_port_info->remote_port;	// 获取对方端口号
		T_arg->proto.tcp->remote_ip[0] = P_port_info->remote_ip[0];	// 获取对方的IP地址
		T_arg->proto.tcp->remote_ip[1] = P_port_info->remote_ip[1];
		T_arg->proto.tcp->remote_ip[2] = P_port_info->remote_ip[2];
		T_arg->proto.tcp->remote_ip[3] = P_port_info->remote_ip[3];
		//os_memcpy(T_arg->proto.tcp->remote_ip,P_port_info->remote_ip,4);	// 内存拷贝
	}
	*/

	//--------------------------------------------------------------------
	OLED_ShowIP(24,6,T_arg->proto.tcp->remote_ip);	// 显示远端主机IP地址
	//--------------------------------------------------------------------

	//【TCP通信是面向连接的，向远端主机回应时可直接使用T_arg结构体指针指向的IP信息】
	//-----------------------------------------------------------------------------------------------
	espconn_send(T_arg,"ESP8266_WIFI_Recv_OK",os_strlen("ESP8266_WIFI_Recv_OK"));	// 向对方发送应答
}
//=========================================================================================================


// TCP连接断开成功的回调函数
//================================================================
void ICACHE_FLASH_ATTR ESP8266_TCP_Disconnect_Cb_JX(void *arg)
{
	os_printf("\nESP8266_TCP_Disconnect_OK\n");
}
//================================================================


// TCP连接建立成功的回调函数
//====================================================================================================================
void ICACHE_FLASH_ATTR ESP8266_TCP_Connect_Cb_JX(void *arg)
{
	espconn_regist_sentcb((struct espconn *)arg, ESP8266_WIFI_Send_Cb_JX);			// 注册网络数据发送成功的回调函数
	espconn_regist_recvcb((struct espconn *)arg, ESP8266_WIFI_Recv_Cb_JX);			// 注册网络数据接收成功的回调函数
	espconn_regist_disconcb((struct espconn *)arg,ESP8266_TCP_Disconnect_Cb_JX);	// 注册成功断开TCP连接的回调函数

	os_printf("\n--------------- ESP8266_TCP_Connect_OK ---------------\n");
}
//====================================================================================================================


// TCP连接异常断开时的回调函数
//====================================================================
void ICACHE_FLASH_ATTR ESP8266_TCP_Break_Cb_JX(void *arg,sint8 err)
{
	os_printf("\nESP8266_TCP_Break\n");
}
//====================================================================


// 定义espconn型结构体
//-----------------------------------------------
//struct espconn ST_NetCon;	// 网络连接结构体

// 初始化网络连接(TCP通信)
//=========================================================================================================
void ICACHE_FLASH_ATTR ESP8266_NetCon_Init_JX()
{
	// 结构体赋值
	//--------------------------------------------------------------------------
	ST_NetCon.type = ESPCONN_TCP ;				// 设置为TCP协议

	ST_NetCon.proto.tcp = (esp_tcp *)os_zalloc(sizeof(esp_tcp));	// 开辟内存

	// 此处无需设置目标IP/端口(ESP8266作为Server，不需要预先知道Client的IP/端口)
	//--------------------------------------------------------------------------
	ST_NetCon.proto.tcp->local_port = 8266 ;	// 设置本地端口
	//ST_NetCon.proto.tcp->remote_port = 8888;	// 设置目标端口
	//ST_NetCon.proto.tcp->remote_ip[0] = 192;	// 设置目标IP地址
	//ST_NetCon.proto.tcp->remote_ip[1] = 168;
	//ST_NetCon.proto.tcp->remote_ip[2] = 8;
	//ST_NetCon.proto.tcp->remote_ip[3] = 47;


	// 注册连接成功回调函数、异常断开回调函数
	//--------------------------------------------------------------------------------------------------
	espconn_regist_connectcb(&ST_NetCon, ESP8266_TCP_Connect_Cb_JX);	// 注册TCP连接成功建立的回调函数
	espconn_regist_reconcb(&ST_NetCon, ESP8266_TCP_Break_Cb_JX);		// 注册TCP连接异常断开的回调函数


	// 创建TCP_server，建立侦听
	//----------------------------------------------------------
	espconn_accept(&ST_NetCon);	// 创建TCP_server，建立侦听

	espconn_regist_time(&ST_NetCon,300, 0); 	//设置超时断开时间。单位=秒，最大值=7200

}
//=========================================================================================================


// IP定时检查的回调函数
//=========================================================================================================
void ICACHE_FLASH_ATTR OS_Timer_IP_cb(void)
{
	struct ip_info ST_ESP8266_IP;	// ESP8266的IP信息
	u8 ESP8266_IP[4];				// ESP8266的IP地址

	u8 S_WIFI_STA_Connect = wifi_station_get_connect_status();


	// 成功接入WIFI【STA模式下，如果开启DHCP(默认)，则ESO8266的IP地址由WIFI路由器自动分配】
	//-------------------------------------------------------------------------------------
	if( S_WIFI_STA_Connect == STATION_GOT_IP )	// 判断是否获取IP
	{
		wifi_get_ip_info(STATION_IF,&ST_ESP8266_IP);	// 获取STA的IP信息
		ESP8266_IP[0] = ST_ESP8266_IP.ip.addr;			// IP地址高八位 == addr低八位
		ESP8266_IP[1] = ST_ESP8266_IP.ip.addr>>8;		// IP地址次高八位 == addr次低八位
		ESP8266_IP[2] = ST_ESP8266_IP.ip.addr>>16;		// IP地址次低八位 == addr次高八位
		ESP8266_IP[3] = ST_ESP8266_IP.ip.addr>>24;		// IP地址低八位 == addr高八位


		OLED_ShowString(0,0,"ESP8266 = STA");
		OLED_ShowString(0,2,"IP:");
		OLED_ShowString(0,4,"Remote  = STA");
		OLED_ShowString(0,6,"IP:");
		//----------------------------------------------------------------
		os_printf("ESP8266_IP = %d.%d.%d.%d\n",ESP8266_IP[0],ESP8266_IP[1],ESP8266_IP[2],ESP8266_IP[3]);
		OLED_ShowIP(24,2,ESP8266_IP);	// OLED显示ESP8266的IP地址
		//----------------------------------------------------------------

		// 接入WIFI成功后，LED快闪3次
		//----------------------------------------------------
		for(; C_LED_Flash<=5; C_LED_Flash++)
		{
			GPIO_OUTPUT_SET(GPIO_ID_PIN(4),(C_LED_Flash%2));
			delay_ms(100);
		}

		os_timer_disarm(&OS_Timer_IP);	// 关闭定时器

		//ESP8266_SNTP_Init_JX();			// 初始化SNTP
		ESP8266_NetCon_Init_JX();		// 初始化网络连接(TCP通信)

	}


	// ESP8266无法连接WIFI
	//------------------------------------------------------------------------------------------------
	else if(	S_WIFI_STA_Connect==STATION_NO_AP_FOUND 	||		// 未找到指定WIFI
				S_WIFI_STA_Connect==STATION_WRONG_PASSWORD 	||		// WIFI密码错误
				S_WIFI_STA_Connect==STATION_CONNECT_FAIL		)	// 连接WIFI失败
	{
		os_timer_disarm(&OS_Timer_IP);			// 关闭定时器

		os_printf("\r\n---- ESP8266 Can't Connect to WIFI-----------\r\n");


		// 微信智能配网设置
		//…………………………………………………………………………………………………………………………
		//wifi_set_opmode(STATION_MODE);		// 设为STA模式							//【第①步】

		smartconfig_set_type(SC_TYPE_AIRKISS); 	// ESP8266配网方式【AIRKISS】			//【第②步】

		smartconfig_start(smartconfig_done);	// 进入【智能配网模式】,并设置回调函数	//【第③步】
		//…………………………………………………………………………………………………………………………
	}
}
//=========================================================================================================

// 软件定时器初始化(ms毫秒)
//========================================================================================
void ICACHE_FLASH_ATTR OS_Timer_IP_Init_JX(u32 time_ms, u8 time_repetitive)
{
	os_timer_disarm(&OS_Timer_IP);	// 关闭定时器
	os_timer_setfn(&OS_Timer_IP,(os_timer_func_t *)OS_Timer_IP_cb, NULL);	// 设置定时器
	os_timer_arm(&OS_Timer_IP, time_ms, time_repetitive);  // 使能定时器
}
//========================================================================================

// LED初始化
//=============================================================================
void ICACHE_FLASH_ATTR LED_Init_JX(void)
{
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO4_U,	FUNC_GPIO4);	// GPIO4设为IO口

	GPIO_OUTPUT_SET(GPIO_ID_PIN(4),1);						// IO4 = 1
}

// GPIO初始化
//=============================================================================
void ICACHE_FLASH_ATTR JDQ_Init_JX(void)
{
	PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12);

	// TODO tcp 多连接测试 ，新建两个客户端进行测试连接情况
	GPIO_OUTPUT_SET(GPIO_ID_PIN(12),1);

}
//=============================================================================


// user_init：entry of user application, init user function here
//=================================================================================================
void ICACHE_FLASH_ATTR user_init(void)
{
	uart_init(115200,115200);	// 初始化串口波特率
	os_delay_us(10000);			// 等待串口稳定
	os_printf("\r\n=================================================\r\n");
	os_printf("\t Project:\t%s\r\n", ProjectName);
	os_printf("\t SDK version:\t%s", system_get_sdk_version());
	os_printf("\r\n=================================================\r\n");

	// OLED显示初始化
	//--------------------------------------------------------
	OLED_Init();
	OLED_ShowString(0,0,"ESP8266 = STA");
	OLED_ShowString(0,2,"IP:");
	OLED_ShowString(0,4,"Remote  = STA");
	OLED_ShowString(0,6,"IP:");
	//－－－－－－－－－－－－－－－－－－－－－－－

	LED_Init_JX();		// LED初始化
	JDQ_Init_JX();		// 继电器初始化

// ESP8266读取【外部Flash】中的【STA参数】(SSID/PASS)，作为STA，连接WIFI
//………………………………………………………………………………………………………………………………
	os_memset(&STA_INFO,0,sizeof(struct station_config));			// STA_INFO = 0
	spi_flash_read(Sector_STA_INFO*4096,(uint32 *)&STA_INFO, 96);	// 读出【STA参数】(SSID/PASS)
	STA_INFO.ssid[31] = 0;		// SSID最后添'\0'
	STA_INFO.password[63] = 0;	// APSS最后添'\0'
	os_printf("\r\nSTA_INFO.ssid=%s\r\nSTA_INFO.password=%s\r\n",STA_INFO.ssid,STA_INFO.password);


	wifi_set_opmode(0x01);					// 设置为STA模式，并保存到Flash
	struct ip_info ST_ESP8266_IP;
	wifi_station_dhcpc_stop();
	IP4_ADDR(&ST_ESP8266_IP.ip,192,168,2,66);
	IP4_ADDR(&ST_ESP8266_IP.netmask,255,255,255,0);
	IP4_ADDR(&ST_ESP8266_IP.gw,192,168,2,1);
	wifi_set_ip_info(STATION_IF,&ST_ESP8266_IP);
	wifi_station_set_config(&STA_INFO);		// 设置STA参数
//	wifi_station_connect();					// ESP8266连接WIFI（这里，此句可省）
//………………………………………………………………………………………………………………………………

	OS_Timer_IP_Init_JX(1000,1);	// 定时查询8266连接WIFI情况
}
//=================================================================================================


/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 ICACHE_FLASH_ATTR user_rf_cal_sector_set(void)
{
    enum flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;

    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        case FLASH_SIZE_64M_MAP_1024_1024:
            rf_cal_sec = 2048 - 5;
            break;
        case FLASH_SIZE_128M_MAP_1024_1024:
            rf_cal_sec = 4096 - 5;
            break;
        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void ICACHE_FLASH_ATTR user_rf_pre_init(void){}

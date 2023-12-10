/*
 **************************************************************************
 * @file 		Atwinc3400.c
 * @brief 		Connecting INDUS board to IoT based cloud platforms for sensor data visualization - Pub Nub Cloud application.
 **************************************************************************
 *
 */
/* Private Includes ---------------------------------------------------------*/
#include "Atwinc3400.h"

/* Private variables ---------------------------------------------------------*/
typedef enum wifi_status {
	WifiStateInit,
	WifiStateWaitingProv,
	WifiStateConnecting,
	WifiStateConnected,
	WifiStateDisConnected
} wifi_status_t;


/** WiFi status variable. */
volatile wifi_status_t gWifiState = WifiStateInit;

/** SysTick counter to avoid busy wait delay. */
volatile uint32_t gu32MsTicks = 0;

/** Global counter delay for timer. */
static uint32_t gu32publishDelay = 0;
static uint32_t gu32subscribeDelay = 0;

/** PubNub global variables. */
static const char PubNubPublishKey[] = MAIN_PUBNUB_PUBLISH_KEY;
static const char PubNubSubscribeKey[] = MAIN_PUBNUB_SUBSCRIBE_KEY;
static char PubNubChannel[] = MAIN_PUBNUB_CHANNEL;
static pubnub_t *pPubNubCfg;

static uint8_t scan_request_index = 0;
/** Number of APs found. */
static uint8_t num_founded_ap = 0;


tstrWifiInitParam wifiInitParam;
int8_t s8InitStatus;
uint8 mac_addr[6];
uint8 u8IsMacAddrValid;
double temperature = 0;
double humidity=0;
uint16_t light = 0;
char buf[256] = {0};
	
int8_t ret;
 tstrWifiInitParam param;


  /*
   *  **************************************************************************
   * @brief Host Interface(HIF) between host driver and ATWINC firmware
   * @param GPIO_Pin
   * @return None
   *  **************************************************************************
   */
  void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == CONF_WINC_SPI_INT_PIN)
    {
        isr();
    }
}

  /*
   *  **************************************************************************
   * @brief Wrapper to get Temprature data from sensor
   * @param none
   * @return sensor value
   *  **************************************************************************
   */
uint16_t GetTempratureShtc3(){
char buffer8[27]={0};
	uint16_t buffer7 = 0;
	uint16_t buffer9 = 0;
buffer7 = Read_Temprature();
	buffer9 = (175 * (float)buffer7 / 65536.0f - 45.0f)-4;
	  HAL_Delay(500);
return buffer9;
}
//void GetHumidityShtc3(){
//char buffer8[27]={0};
//uint16_t buffer17 = 0;
//uint16_t buffer27 = 0;
//buffer27 = Read_Humidity();
//	buffer17 = 100 * (float)buffer27 / 65536.0f;
//
//	  HAL_Delay(500);
//	 return buffer17;
//}

/*
 *  **************************************************************************
 * @brief Wrapper to get Humidity data from sensor
 * @param none
 * @return sensor value
 *  **************************************************************************
 */
uint16_t GetHumidyShtc3(){
char buffer8[27]={0};
	uint16_t buffer10=0;
	uint16_t buffer7 = 0;
buffer7 = Read_Humidity();
	buffer10 = (100 * (float)buffer7 / 65536.0f)+10;
	  HAL_Delay(500);
return buffer10;
}

/**
 * **************************************************************************
 * \brief Callback to get the Socket event.
 *
 * \param[in] Socket descriptor.
 * \param[in] msg_type type of Socket notification. Possible types are:
 *  - [SOCKET_MSG_CONNECT](@ref SOCKET_MSG_CONNECT)
 *  - [SOCKET_MSG_BIND](@ref SOCKET_MSG_BIND)
 *  - [SOCKET_MSG_LISTEN](@ref SOCKET_MSG_LISTEN)
 *  - [SOCKET_MSG_ACCEPT](@ref SOCKET_MSG_ACCEPT)
 *  - [SOCKET_MSG_RECV](@ref SOCKET_MSG_RECV)
 *  - [SOCKET_MSG_SEND](@ref SOCKET_MSG_SEND)
 *  - [SOCKET_MSG_SENDTO](@ref SOCKET_MSG_SENDTO)
 *  - [SOCKET_MSG_RECVFROM](@ref SOCKET_MSG_RECVFROM)
 * \param[in] msg_data A structure contains notification informations.
 * **************************************************************************
 */
static void m2m_tcp_socket_handler(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
	handle_tcpip(sock, u8Msg, pvMsg);
}

/**
 * **************************************************************************
 * \brief Callback of gethostbyname function.
 *
 * \param[in] doamin_name Domain name.
 * \param[in] server_ip IP of server.
 * **************************************************************************
 */
static void socket_resolve_cb(uint8_t *hostName, uint32_t hostIp)
{
	printf("socket_resolve_cb: %s resolved with IP %d.%d.%d.%d\r\n",
			hostName,
			(int)IPV4_BYTE(hostIp, 0), (int)IPV4_BYTE(hostIp, 1),
			(int)IPV4_BYTE(hostIp, 2), (int)IPV4_BYTE(hostIp, 3));
	handle_dns_found((char *)hostName, hostIp);
}


/**
 * **************************************************************************
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType type of Wi-Fi notification. Possible types are:
 *  - [M2M_WIFI_RESP_CURRENT_RSSI](@ref M2M_WIFI_RESP_CURRENT_RSSI)
 *  - [M2M_WIFI_RESP_CON_STATE_CHANGED](@ref M2M_WIFI_RESP_CON_STATE_CHANGED)
 *  - [M2M_WIFI_RESP_CONNTION_STATE](@ref M2M_WIFI_RESP_CONNTION_STATE)
 *  - [M2M_WIFI_RESP_SCAN_DONE](@ref M2M_WIFI_RESP_SCAN_DONE)
 *  - [M2M_WIFI_RESP_SCAN_RESULT](@ref M2M_WIFI_RESP_SCAN_RESULT)
 *  - [M2M_WIFI_REQ_WPS](@ref M2M_WIFI_REQ_WPS)
 *  - [M2M_WIFI_RESP_IP_CONFIGURED](@ref M2M_WIFI_RESP_IP_CONFIGURED)
 *  - [M2M_WIFI_RESP_IP_CONFLICT](@ref M2M_WIFI_RESP_IP_CONFLICT)
 *  - [M2M_WIFI_RESP_P2P](@ref M2M_WIFI_RESP_P2P)
 *  - [M2M_WIFI_RESP_AP](@ref M2M_WIFI_RESP_AP)
 *  - [M2M_WIFI_RESP_CLIENT_INFO](@ref M2M_WIFI_RESP_CLIENT_INFO)
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters
 * (if any). It should be casted to the correct data type corresponding to the
 * notification type. Existing types are:
 *  - tstrM2mWifiStateChanged
 *  - tstrM2MWPSInfo
 *  - tstrM2MP2pResp
 *  - tstrM2MAPResp
 *  - tstrM2mScanDone
 *  - tstrM2mWifiscanResult
 *  **************************************************************************
 */
static void m2m_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType) {
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED) {
			printf("m2m_wifi_state: M2M_WIFI_RESP_CON_STATE_CHANGED: CONNECTED\r\n");
		} else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED) {
			printf("m2m_wifi_state: M2M_WIFI_RESP_CON_STATE_CHANGED: DISCONNECTED\r\n");
			if (WifiStateConnected == gWifiState) {
				gWifiState = WifiStateDisConnected;
				m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
						MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
			}
		}

		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		printf("m2m_wifi_state: M2M_WIFI_REQ_DHCP_CONF: IP is %u.%u.%u.%u\r\n",
				pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		gWifiState = WifiStateConnected;

		break;
	}

	default:
	{
		break;
	}
	}
}


/*
 * **************************************************************************
 * @brief Set Device Name To MAC.
 * @param   none
 * @return   None
 * **************************************************************************
 */
static void set_dev_name_to_mac(uint8 *name, uint8 *mac_addr)
{
	/* Name must be in the format WINC1500_00:00 */
	uint16 len;

	len = m2m_strlen(name);
	if (len >= 5) {
		name[len - 1] = HEX2ASCII((mac_addr[5] >> 0) & 0x0f);
		name[len - 2] = HEX2ASCII((mac_addr[5] >> 4) & 0x0f);
		name[len - 4] = HEX2ASCII((mac_addr[4] >> 0) & 0x0f);
		name[len - 5] = HEX2ASCII((mac_addr[4] >> 4) & 0x0f);
	}
}


static void configure_button_led(void)
{
/*	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED0_PIN, &pin_conf);
	port_pin_set_output_level(LED0_PIN, LED0_INACTIVE);*/
}

static void configure_light_sensor(void)
{
/*	struct adc_config config_adc;
	adc_get_config_defaults(&config_adc);

	config_adc.gain_factor = ADC_GAIN_FACTOR_DIV2;
	config_adc.clock_prescaler = ADC_CLOCK_PRESCALER_DIV512;
	config_adc.reference = ADC_REFERENCE_INTVCC1;
	config_adc.positive_input = ADC_POSITIVE_INPUT_PIN0;
	config_adc.resolution = ADC_RESOLUTION_12BIT;
	config_adc.clock_source = GCLK_GENERATOR_0;
	adc_init(&adc_instance, ADC, &config_adc);
	adc_enable(&adc_instance);*/
}




/*
 * **************************************************************************
 * @brief ATWINC Host driver Initialization
 * @param None
 * @return None
 * **************************************************************************
 */
void Get_WiFi_Init(){

	nm_bsp_init();

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&wifiInitParam, 0, sizeof(tstrWifiInitParam));
	wifiInitParam.pfAppWifiCb = m2m_cb;

	/* Initialize WINC1500 Wi-Fi driver with data and status callbacks. */
	s8InitStatus = m2m_wifi_init(&wifiInitParam);
	if (M2M_SUCCESS != s8InitStatus) {
		printf("main: m2m_wifi_init call error!\r\n");
		while (1) {
		}
	}
	else
		printf("\n\rmain: m2m_wifi_initialize (%d)\r\n", ret);
}

/*
 * **************************************************************************
 * @brief Connect to AP using Wi-Fi settings from main.h.
 * @param None
 * @return None
 * **************************************************************************
 */
void Wifi_Get_Connect()
{
/* Connect to AP using Wi-Fi settings from main.h. */
	printf("main: Wi-Fi connecting to AP using hardcoded credentials...\r\n");
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID),
	MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
  
}


/*
 * **************************************************************************
 * @brief registerSocketCallback.
 * @param None
 * @return None
 * **************************************************************************
 */
void Register_Socket_Callback()
{
		registerSocketCallback(m2m_tcp_socket_handler, socket_resolve_cb);
}


/*
 * **************************************************************************
 * @brief Read MAC address to customize device name and AP name if enabled.
 * @param None
 * @return None
 * **************************************************************************
 */
void Get_MAC_address()
{
	
	/* Read MAC address to customize device name and AP name if enabled. */
	m2m_wifi_get_otp_mac_address(mac_addr, &u8IsMacAddrValid);
	
	if (!u8IsMacAddrValid) {
		printf("main: MAC address fuse bit has not been configured!\r\n");
		printf("main: Use m2m_wifi_set_mac_address() API to set MAC address via software.\r\n");
		while (1) {
		}
	}
	m2m_wifi_get_mac_address(mac_addr);
}

/*
 * **************************************************************************
 * @brief   Set_Dev_Name_To_MAC
 * @param   None
 * @return  None
 * **************************************************************************
 */
void Set_Dev_Name_To_MAC()
{
	set_dev_name_to_mac((uint8 *)PubNubChannel, mac_addr);
}


/*
 * **************************************************************************
 * @brief   Initialize PubNub API.
 * @param   None
 * @return  None
 * **************************************************************************
 */
void Initialize_Pub_Nub_API()
{
	
	printf("\r\n");

	/* Initialize PubNub API. */
	printf("\n\rmain: PubNub configured with following settings:");
	printf("\n\rmain:  - Publish key: \"%s\", Subscribe key: \"%s\", Channel: \"%s\".\n\r",
	PubNubPublishKey, PubNubSubscribeKey, PubNubChannel);
	pPubNubCfg = pubnub_get_ctx(0);
	pubnub_init(pPubNubCfg, PubNubPublishKey, PubNubSubscribeKey);
}


/*
 * **************************************************************************
 * @brief   Subscribe at the beginning and re-subscribe after every publish.
 * @param   None
 * @return  None
 * **************************************************************************
 */
void Pub_Nub_Subscribe()
{
			/* Device is connected to AP. */
		if (gWifiState == WifiStateConnected) {
			/* PubNub: read event from the cloud. */
			if (pPubNubCfg->state == PS_IDLE) {
				/* Subscribe at the beginning and re-subscribe after every publish. */
				if ((pPubNubCfg->trans == PBTT_NONE) ||
				    (pPubNubCfg->trans == PBTT_PUBLISH && pPubNubCfg->last_result == PNR_OK)) {
					printf("main: subscribe event, PNR_OK\r\n");
					pubnub_subscribe(pPubNubCfg, PubNubChannel);				
				}
HAL_Delay(5000);
				/* Process any received messages from the channel we subscribed. */
				while (1) {
					char const *msg = pubnub_get(pPubNubCfg);
					if (NULL == msg) {
						/* No more message to process. */
						break;
					}

					if (0 == (strncmp(&msg[2], "led", strlen("led")))) {
						/* LED control message. */
						printf("main: received LED control message: %s\r\n", msg);
						if (0 == (strncmp(&msg[8], "on", strlen("on")))) {
						//	port_pin_set_output_level(LED0_PIN, LED0_ACTIVE);
						} else if (0 == (strncmp(&msg[8], "off", strlen("off")))) {
							//port_pin_set_output_level(LED0_PIN, LED0_INACTIVE);
						}
					} else {
						/* Any other type of JSON message. */
						printf("main: received message: %s\r\n", msg);
					}
				}
HAL_Delay(5000);
				/* Subscribe to receive pending messages. */
			//	if (gu32MsTicks - gu32subscribeDelay > MAIN_PUBNUB_SUBSCRIBE_INTERVAL) {
					gu32subscribeDelay = gu32MsTicks;
					printf("main: subscribe event, interval.\r\n");
					pubnub_subscribe(pPubNubCfg, PubNubChannel);
			//	}
			}
HAL_Delay(5000);
			/* Publish the temperature measurements periodically. */
			//if (gu32MsTicks - gu32publishDelay > MAIN_PUBNUB_PUBLISH_INTERVAL) 
			
			//	gu32publishDelay = gu32MsTicks;
			//	adc_start_conversion(&adc_instance);
				temperature =25; //at30tse_read_temperature();
			//	adc_read(&adc_instance, &light);
				sprintf(buf, "{\"device\":\"%s\", \"temperature\":\"%d\", \"humidity\":\"%d\"}",
						PubNubChannel,
						(int)GetTempratureShtc3(),(int)GetHumidyShtc3());
				printf("main: publish event: {%s}\r\n", buf);
				close(pPubNubCfg->tcp_socket);
				pPubNubCfg->tcp_socket = -1;
				pPubNubCfg->state = PS_IDLE;
				pPubNubCfg->last_result = PNR_IO_ERROR;
				pubnub_publish(pPubNubCfg, PubNubChannel, buf);
						
						Delay(2000);
			//}
		}
}




/*
 * **************************************************************************
 * @brief This function provides minimum delay (in milliseconds)
 * @param int a describe the delay time in milliseconds
 * @return None
 * **************************************************************************
 */
Delay(int a)
{
HAL_Delay(a);
}


/**
 * **************************************************************************
  * @brief Send an amount of data in blocking mode.
  * @param UART   UART handle.
  * @param pData  String.
  * @param Size    Amount of data elements (u8 or u16) to be sent.
  * @param Timeout Timeout duration.
  * @retval None
  * **************************************************************************
  */
void UART_Transmit(UART_HandleTypeDef *UART,char TxData[],uint16_t Size)
{
	HAL_UART_Transmit(UART, (uint8_t *)TxData,Size,100);

}

/**
 *  **************************************************************************
  * @brief Putchar Prototype for printf functionality
  * @retval None
  *  **************************************************************************
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART3 and Loop until the end of transmission */
  UART_Transmit(&huart1, (uint8_t *)&ch, 1);

  return ch;
}

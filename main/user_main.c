#include <stdio.h>
#include "esp_system.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_err.h"
#include "internal/esp_wifi_internal.h"
#include "nvs_flash.h"
#include "rom/ets_sys.h"
#include "driver/uart.h"
#include "tcpip_adapter.h"
#include "esp_smartconfig.h"
#include "smartconfig_ack.h"
#include "airkiss.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

/**
 *    由于 esp-idf esp8266芯片 rtos3.0 sdk 乐鑫没做微信近场发现的功能，于是动动手指做起来！
 *    这是微信airkiss配网以及近场发现的功能的demo示范，亲测可以配网成功以及近场发现！
 *    有任何技术问题邮箱： 870189248@qq.com
 *    本人GitHub仓库：https://github.com/xuhongv
 *    本人博客：https://blog.csdn.net/xh870189248
 *    
 *    注意事项 ------> airkiss2.0 仅可以近场发现，不可以自定义双向通讯！仅可以自定义消息发送给微信客户端！
 *    下面的deviceInfo可以自定义，但是为了兼容ios手机，必须要base64编码后传给微信静态库SDK！本demo没做base64加密，所以在ios上近场发现失败！
 *
 * 
 **/

static EventGroupHandle_t wifi_event_group;
static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;
static const int AIRKISS_DONE_BIT = BIT2;

static xTaskHandle handleLlocalFind = NULL;
static const char *TAG = "xAirKiss";

//airkiss
#define COUNTS_BOACAST 30            //发包次数，微信建议20次以上
#define ACCOUNT_ID "gh_083fe269017c" //微信公众号
#define LOCAL_UDP_PORT 12476         //固定端口号
uint8_t deviceInfo[60] = {"5351722"};  //设备ID，也可以任意设置。

int sock_fd;

const airkiss_config_t akconf = {
    (airkiss_memset_fn)&memset,
    (airkiss_memcpy_fn)&memcpy,
    (airkiss_memcmp_fn)&memcmp,
    0,
};

static void TaskCreatSocket(void *pvParameters)
{

    char rx_buffer[128];
    char addr_str[128];
    uint8_t lan_buf[300];
    uint16_t lan_buf_len;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;
    int sock_server; /* server socked */
    int err;
    int counts = 0;

    sock_server = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_server == -1)
    {
        printf("failed to create sock_fd!\n");
        vTaskDelete(NULL);
    }

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY); // inet_addr("255.255.255.255");
    server_addr.sin_port = htons(LOCAL_UDP_PORT);

    err = bind(sock_server, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (err == -1)
    {
        vTaskDelete(NULL);
    }

    struct sockaddr_in sourceAddr;
    socklen_t socklen = sizeof(sourceAddr);
    while (1)
    {
        memset(rx_buffer, 0, sizeof(rx_buffer));
        int len = recvfrom(sock_server, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&sourceAddr, &socklen);

        ESP_LOGI(TAG, "IP:%s:%d", (char *)inet_ntoa(sourceAddr.sin_addr), htons(sourceAddr.sin_port));
        //ESP_LOGI(TAG, "Received %s ", rx_buffer);

        // Error occured during receiving
        if (len < 0)
        {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            break;
        }
        // Data received
        else
        {
            rx_buffer[len] = 0;                                                // Null-terminate whatever we received and treat like a string
            airkiss_lan_ret_t ret = airkiss_lan_recv(rx_buffer, len, &akconf); //检测是否为微信发的数据包
            airkiss_lan_ret_t packret;
            switch (ret)
            {
            case AIRKISS_LAN_SSDP_REQ:

                lan_buf_len = sizeof(lan_buf);
                //开始组装打包
                packret = airkiss_lan_pack(AIRKISS_LAN_SSDP_NOTIFY_CMD, ACCOUNT_ID, deviceInfo, 0, 0, lan_buf, &lan_buf_len, &akconf);
                if (packret != AIRKISS_LAN_PAKE_READY)
                {
                    ESP_LOGE(TAG, "Pack lan packet error!");
                    continue;
                }
                ESP_LOGI(TAG, "Pack lan packet ok !");
                //发送至微信客户端
                int err = sendto(sock_server, (char *)lan_buf, lan_buf_len, 0, (struct sockaddr *)&sourceAddr, sizeof(sourceAddr));
                if (err < 0)
                {
                    ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                }
                break;
            default:
                break;
            }
        }
    }

___shutDownAirkissTask:
{

    shutdown(sock_fd, 0);
    close(sock_fd);
    vTaskDelete(&handleLlocalFind);
}
}
/**
 * @description: 关闭进程
 * @param {type} 
 * @return: 
 */
void shutDownAirkissTask()
{
    shutdown(sock_fd, 0);
    close(sock_fd);
    vTaskDelete(&handleLlocalFind);
}

bool startAirkissTask()
{
    int ret = pdFAIL;
    if (handleLlocalFind == NULL)
        ret = xTaskCreate(TaskCreatSocket, "TaskCreatSocket", 1024 * 3, NULL, 4, &handleLlocalFind);

    if (ret != pdPASS)
    {
        printf("create airkiss thread failed.\n");
        return false;
    }
    else
    {
        return true;
    }
}

void smartconfig_example_task(void *parm);

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id)
    {
    case SYSTEM_EVENT_STA_START:
        xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void sc_callback(smartconfig_status_t status, void *pdata)
{
    switch (status)
    {
    case SC_STATUS_LINK:
    {
        wifi_config_t *wifi_config = pdata;
        ESP_LOGI(TAG, "SSID:%s", wifi_config->sta.ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", wifi_config->sta.password);
        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, wifi_config));
        ESP_ERROR_CHECK(esp_wifi_connect());
    }
    break;
    case SC_STATUS_LINK_OVER:
        ESP_LOGI(TAG, "SC_STATUS_LINK_OVER");
        //这里乐鑫回调目前在master分支已区分是否为微信配网还是esptouch配网，当airkiss配网才近场回调！
        if (pdata != NULL)
        {
            sc_callback_data_t *sc_callback_data = (sc_callback_data_t *)pdata;
            switch (sc_callback_data->type)
            {
            case SC_ACK_TYPE_ESPTOUCH:
                ESP_LOGI(TAG, "Phone ip: %d.%d.%d.%d", sc_callback_data->ip[0], sc_callback_data->ip[1], sc_callback_data->ip[2], sc_callback_data->ip[3]);
                ESP_LOGI(TAG, "TYPE: ESPTOUCH");
                xEventGroupSetBits(wifi_event_group, ESPTOUCH_DONE_BIT);
                break;
            case SC_ACK_TYPE_AIRKISS:
                ESP_LOGI(TAG, "TYPE: AIRKISS");
                xEventGroupSetBits(wifi_event_group, AIRKISS_DONE_BIT);
                break;
            default:
                ESP_LOGE(TAG, "TYPE: ERROR");
                xEventGroupSetBits(wifi_event_group, ESPTOUCH_DONE_BIT);
                break;
            }
        }

        break;
    default:
        break;
    }
}

void smartconfig_example_task(void *parm)
{
    EventBits_t uxBits;
    ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH_AIRKISS));
    ESP_ERROR_CHECK(esp_smartconfig_start(sc_callback));
    while (1)
    {
        uxBits = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT | AIRKISS_DONE_BIT, true, false, portMAX_DELAY);
        if (uxBits & CONNECTED_BIT)
        {
            ESP_LOGI(TAG, "WiFi Connected to ap");
        }

        if (uxBits & AIRKISS_DONE_BIT)
        {
            ESP_LOGI(TAG, "smartconfig over , start find device");
            esp_smartconfig_stop();
            startAirkissTask();
            ESP_LOGI(TAG, "getAirkissVersion %s", airkiss_version());
            vTaskDelete(NULL);
        }

        if (uxBits & ESPTOUCH_DONE_BIT)
        {
            ESP_LOGI(TAG, "smartconfig over , but don't find device by airkiss...");
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }
    }
}

/******************************************************************************
 * FunctionName : app_main
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    printf("\n\n-------------------------------- Get Systrm Info------------------------------------------\n");
    //获取IDF版本
    printf("     SDK version:%s\n", esp_get_idf_version());
    //获取芯片可用内存
    printf("     esp_get_free_heap_size : %d  \n", esp_get_free_heap_size());
    //获取从未使用过的最小内存
    printf("     esp_get_minimum_free_heap_size : %d  \n", esp_get_minimum_free_heap_size());
    //获取芯片的内存分布，返回值具体见结构体 flash_size_map
    printf("     system_get_flash_size_map(): %d \n", system_get_flash_size_map());
    //获取mac地址（station模式）
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    printf("esp_read_mac(): %02x:%02x:%02x:%02x:%02x:%02x \n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    printf("--------------------------------------------------------------------------\n\n");
    initialise_wifi();
}

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
#include "app_mqtt_handle.h"
#include "rom/ets_sys.h"
#include "driver/uart.h"
#include "tcpip_adapter.h"
#include "esp_smartconfig.h"
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
 **/

static EventGroupHandle_t wifi_event_group;
static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;

static xTaskHandle handleLlocalFind = NULL;
static const char *TAG = "xAirKiss";

//airkiss
#define COUNTS_BOACAST 30               //发包次数，微信建议20次以上
#define ACCOUNT_ID "gh_083fe269017c"    //微信公众号
#define LOCAL_UDP_PORT 12476            //固定端口号
uint8_t deviceInfo[60] = {"xuhongYss"}; //设备ID

int sock_fd;
struct sockaddr_in client_addr;

const airkiss_config_t akconf = {
    (airkiss_memset_fn)&memset,
    (airkiss_memcpy_fn)&memcpy,
    (airkiss_memcmp_fn)&memcmp,
    0,
};

uint8_t lan_buf[300];
uint16_t lan_buf_len;
static void TaskLocalBoacast(void *pvParameters)
{

    int err;
    int count = 0;
    while (1)
    {

        lan_buf_len = sizeof(lan_buf);
        airkiss_lan_ret_t ret = airkiss_lan_pack(AIRKISS_LAN_SSDP_NOTIFY_CMD, ACCOUNT_ID, deviceInfo, 0, 0, lan_buf, &lan_buf_len, &akconf);

        if (ret != AIRKISS_LAN_PAKE_READY)
        {
            printf("Pack lan packet error! \n");
            vTaskDelete(NULL);
        }

        err = sendto(sock_fd, (char *)lan_buf, lan_buf_len, 0, (struct sockaddr *)&client_addr, sizeof(client_addr));
        ESP_LOGI(TAG, "[SY]  send local !  %d ", count);

        if (err < 0)
        {
            printf("[SY] failed to send local ! ... \n");
        }
        else
        {
            count++;
            if (count > COUNTS_BOACAST)
            {
                vTaskDelete(NULL);
            }
        }
        vTaskDelay(1500 / portTICK_RATE_MS);
    }
}

static void TaskCreatSocket(void *pvParameters)
{

    char rx_buffer[128];
    char addr_str[128];

    sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd == -1)
    {
        printf("[xuhong] failed to create sock_fd!\n");
        vTaskDelete(NULL);
    }

    memset(&client_addr, 0, sizeof(client_addr));
    client_addr.sin_family = AF_INET;
    client_addr.sin_addr.s_addr = inet_addr("255.255.255.255");
    client_addr.sin_port = htons(LOCAL_UDP_PORT);

    xTaskCreate(TaskLocalBoacast, "TaskLocalBoacast", 1024, NULL, 5, NULL);

    while (1)
    {
        //sprintf(udp_msg,"");
        vTaskDelay(1000 / portTICK_RATE_MS);

        struct sockaddr_in sourceAddr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(sourceAddr);
        int len = recvfrom(sock_fd, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&sourceAddr, &socklen);

        // Error occured during receiving
        if (len < 0)
        {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            break;
        }
        // Data received
        else
        {
            rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
            ESP_LOGI(TAG, "Received %d bytes : %s", len, rx_buffer);

            uint16_t i;
            airkiss_lan_ret_t ret = airkiss_lan_recv(rx_buffer, len, &akconf);
            airkiss_lan_ret_t packret;
            switch (ret)
            {
            case AIRKISS_LAN_SSDP_REQ:

                packret = airkiss_lan_pack(AIRKISS_LAN_SSDP_RESP_CMD,
                                           ACCOUNT_ID, deviceInfo, 0, 0, lan_buf, &lan_buf_len, &akconf);

                if (packret != AIRKISS_LAN_PAKE_READY)
                {
                    printf("Pack lan packet error!");
                    return;
                }

                inet_ntoa_r(((struct sockaddr_in *)&sourceAddr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);

                int err = sendto(sock_fd, lan_buf, lan_buf_len, 0, (struct sockaddr *)&sourceAddr, sizeof(sourceAddr));

                if (err < 0)
                {
                    ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
                    break;
                }

                break;
            default:
                printf("Pack is not ssdq req!%d\r\n", ret);
                break;
            }
        }
    }

    vTaskDelete(NULL);
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

void shutDownAirkissTask()
{
    shutdown(sock_fd, 0);
    close(sock_fd);
    vTaskDelete(&handleLlocalFind);
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
        wifi_config_t *wifi_config = pdata;
        ESP_LOGI(TAG, "SSID:%s", wifi_config->sta.ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", wifi_config->sta.password);
        ESP_ERROR_CHECK(esp_wifi_disconnect());
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, wifi_config));
        ESP_ERROR_CHECK(esp_wifi_connect());
        break;
    case SC_STATUS_LINK_OVER:
        ESP_LOGI(TAG, "SC_STATUS_LINK_OVER");
        //这里乐鑫回调无法给我做是否为微信配网还是 esptouch配网，所以我全部统一为airkiss配网（不认识的全部贾玲处理，哈哈）
        if (pdata != NULL)
        {
            uint8_t phone_ip[4] = {0};
            memcpy(phone_ip, (uint8_t *)pdata, 4);
            ESP_LOGI(TAG, "lcoal phone ip : %d.%d.%d.%d ", phone_ip[0], phone_ip[1], phone_ip[2], phone_ip[3]);
        }
        xEventGroupSetBits(wifi_event_group, ESPTOUCH_DONE_BIT);
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
        uxBits = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
        if (uxBits & CONNECTED_BIT)
        {
            ESP_LOGI(TAG, "WiFi Connected to ap");
        }
        if (uxBits & ESPTOUCH_DONE_BIT)
        {
            ESP_LOGI(TAG, "smartconfig over");
            esp_smartconfig_stop();
            startAirkissTask();
            ESP_LOGI(TAG, "getAirkissVersion %s", airkiss_version());
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

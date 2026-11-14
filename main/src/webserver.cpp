#include "webserver.hpp"
#include <cstring>
#include "esp_log.h"
#include "esp_http_server.h"

extern volatile uint8_t autoMode;

static const char* TAG = "WEB_SERVER";

static const char INDEX_HTML[] =
"<!DOCTYPE html><html lang='pl'><head><meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
"<title>ESP32-S3 Sterowanie</title>"
"<style>"
"body{font-family:sans-serif;text-align:center;background:#121212;color:#eee;margin-top:30px;}"
".btn-auto{padding:14px 24px;font-size:16px;margin-bottom:15px;border:none;border-radius:8px;background:#424242;color:#fff;cursor:pointer;font-weight:bold;}"
"button{padding:18px 30px;font-size:18px;margin:8px;border:none;border-radius:8px;background:#2979ff;color:#fff;cursor:pointer;}"
"button:active{background:#1565c0;}"
"#st{margin-top:20px;color:#00e676;font-weight:bold;}"
"</style></head><body>"
"<h2>ESP32-S3 Panel</h2>"
"<div><button id='autoBtn' class='btn-auto' onclick='toggleAuto()'>Tryb Auto: Ładowanie...</button></div>"
"<button onclick=\"cmd('fwd')\">Przód</button><br>"
"<button onclick=\"cmd('left')\">Lewo</button>"
"<button onclick=\"cmd('stop')\" style='background:#d50000;'>STOP</button>"
"<button onclick=\"cmd('right')\">Prawo</button><br>"
"<button onclick=\"cmd('back')\">Tył</button>"
"<div id='st'>Gotowy</div>"
"<script>"
"function updateUI(isOn) {"
"  let btn = document.getElementById('autoBtn');"
"  btn.style.background = isOn ? '#00e676' : '#424242';"
"  btn.style.color = isOn ? '#121212' : '#fff';"
"  btn.innerText = isOn ? 'Tryb Auto: WŁ' : 'Tryb Auto: WYŁ';"
"}"
"window.onload = function() {"
"  fetch('/status').then(r=>r.text()).then(t => updateUI(t === 'on'));"
"};"
"function toggleAuto() {"
"  let btn = document.getElementById('autoBtn');"
"  let willBeOn = !btn.innerText.includes('WŁ');"
"  let val = willBeOn ? 'auto_on' : 'auto_off';"
"  fetch('/cmd?dir=' + val).then(r=>r.text()).then(t => {"
"    updateUI(willBeOn);"
"    document.getElementById('st').innerText = 'Stan: ' + t;"
"  });"
"}"
"function cmd(c) {"
"  fetch('/cmd?dir=' + c).then(r => r.text()).then(t => {"
"    document.getElementById('st').innerText = 'Odebrano: ' + t;"
"  });"
"}"
"</script></body></html>";

static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t status_get_handler(httpd_req_t *req) {
    const char* resp = autoMode ? "on" : "off";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static esp_err_t cmd_get_handler(httpd_req_t *req) {
    char query[64];
    char dir[16];

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "dir", dir, sizeof(dir)) == ESP_OK) {
            ESP_LOGI(TAG, "Received command from web: %s", dir);

            if (strcmp(dir, "auto_on") == 0) {
                autoMode = 1;
                ESP_LOGI(TAG, "Mode AUTO on");
            } else if (strcmp(dir, "auto_off") == 0) {
                autoMode = 0;
                ESP_LOGI(TAG, "Mode AUTO off");
            }

            httpd_resp_send(req, dir, HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }
    }
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No dir parameter.");
    return ESP_FAIL;
}

void WebServer::start() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = nullptr;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root_uri = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler, .user_ctx = nullptr };
        httpd_register_uri_handler(server, &root_uri);

        httpd_uri_t status_uri = { .uri = "/status", .method = HTTP_GET, .handler = status_get_handler, .user_ctx = nullptr };
        httpd_register_uri_handler(server, &status_uri);

        httpd_uri_t cmd_uri = { .uri = "/cmd", .method = HTTP_GET, .handler = cmd_get_handler, .user_ctx = nullptr };
        httpd_register_uri_handler(server, &cmd_uri);
        
        ESP_LOGI(TAG, "HTTP server creation success");
    }
}
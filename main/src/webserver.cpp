#include "webserver.hpp"
#include "robot.hpp"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include "esp_log.h"
#include "esp_http_server.h"

extern volatile uint8_t autoMode;
extern Robot robot;

static const char* TAG = "WEB_SERVER";

static const char INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html><html lang='pl'><head><meta charset='UTF-8'>
<meta name='viewport' content='width=device-width, initial-scale=1.0'>
<title>ESP32-S3 Sterowanie & ToF</title>
<style>
body{font-family:sans-serif;text-align:center;background:#121212;color:#eee;margin:20px;}
.btn-auto{padding:14px 24px;font-size:16px;margin-bottom:15px;border:none;border-radius:8px;background:#424242;color:#fff;cursor:pointer;font-weight:bold;}
button{padding:14px 24px;font-size:16px;margin:6px;border:none;border-radius:8px;background:#2979ff;color:#fff;cursor:pointer;}
button:active{background:#1565c0;}
#st{margin:15px 0;color:#00e676;font-weight:bold;}
.panel{background:#1e1e1e;border:1px solid #333;border-radius:10px;padding:15px;margin:15px auto;max-width:650px;}
.cards{display:flex;justify-content:space-around;flex-wrap:wrap;gap:10px;}
.card{background:#262626;padding:12px;border-radius:8px;min-width:160px;}
.val{font-size:24px;font-weight:bold;color:#00e676;margin:6px 0;}
.off{color:#ff5252;}
table{width:100%;border-collapse:collapse;margin-top:10px;}
th,td{padding:6px;text-align:center;}
input[type=number]{width:70px;padding:5px;background:#333;color:#fff;border:1px solid #555;border-radius:4px;text-align:center;}
</style></head><body>
<h2>ROOMBOBER - ESP32-S3 Panel</h2>

<div class="panel">
    <div><button id='autoBtn' class='btn-auto' onclick='toggleAuto()'>Tryb Auto: Ładowanie...</button></div>
    <button onclick="cmd('fwd')">Przód</button><br>
    <button onclick="cmd('left')">Lewo</button>
    <button onclick="cmd('stop')" style='background:#d50000;'>STOP</button>
    <button onclick="cmd('right')">Prawo</button><br>
    <button onclick="cmd('back')">Tył</button>
    <div id='st'>Gotowy</div>
</div>

<div class="panel">
    <h3>Odczyty Czujników ToF (VL53L0X) & Enkoderów</h3>
    <div class="cards">
        <div class="card"><b>Lewy (S0)</b><div id="d0" class="val">-- mm</div><small id="ok0">--</small></div>
        <div class="card"><b>Środek (S1)</b><div id="d1" class="val">-- mm</div><small id="ok1">--</small></div>
        <div class="card"><b>Prawy (S2)</b><div id="d2" class="val">-- mm</div><small id="ok2">--</small></div>
    </div>
    <p style="margin-top:12px;">Enkoder Lewy: <b id="encL">0</b> imp | Enkoder Prawy: <b id="encR">0</b> imp</p>
</div>

<div class="panel">
    <h3>Konfiguracja Pozycji Czujników (względem środka robota)</h3>
    <table>
        <tr><th>Czujnik</th><th>X [mm] (Przód+)</th><th>Y [mm] (Lewo+)</th><th>Kąt [°] (Lewo+)</th><th>Akcja</th></tr>
        <tr>
            <td>Lewy (0)</td>
            <td><input id="x0" type="number" step="1"></td>
            <td><input id="y0" type="number" step="1"></td>
            <td><input id="a0" type="number" step="1"></td>
            <td><button onclick="saveCfg(0)" style="padding:6px 12px;font-size:14px;">Zapisz</button></td>
        </tr>
        <tr>
            <td>Środek (1)</td>
            <td><input id="x1" type="number" step="1"></td>
            <td><input id="y1" type="number" step="1"></td>
            <td><input id="a1" type="number" step="1"></td>
            <td><button onclick="saveCfg(1)" style="padding:6px 12px;font-size:14px;">Zapisz</button></td>
        </tr>
        <tr>
            <td>Prawy (2)</td>
            <td><input id="x2" type="number" step="1"></td>
            <td><input id="y2" type="number" step="1"></td>
            <td><input id="a2" type="number" step="1"></td>
            <td><button onclick="saveCfg(2)" style="padding:6px 12px;font-size:14px;">Zapisz</button></td>
        </tr>
    </table>
</div>

<script>
function updateUI(isOn) {
    let btn = document.getElementById('autoBtn');
    btn.style.background = isOn ? '#00e676' : '#424242';
    btn.style.color = isOn ? '#121212' : '#fff';
    btn.innerText = isOn ? 'Tryb Auto: WŁ' : 'Tryb Auto: WYŁ';
}

function loadConfigs() {
    fetch('/api/tof').then(r=>r.json()).then(d => {
        for(let i=0; i<3; i++) {
            document.getElementById('x'+i).value = d.cfg[i].x;
            document.getElementById('y'+i).value = d.cfg[i].y;
            document.getElementById('a'+i).value = d.cfg[i].a;
        }
    });
}

window.onload = function() {
    fetch('/status').then(r=>r.text()).then(t => updateUI(t === 'on'));
    loadConfigs();
    setInterval(() => {
        fetch('/api/tof').then(r=>r.json()).then(d => {
            for(let i=0; i<3; i++) {
                document.getElementById('d'+i).innerText = d.dist[i] + ' mm';
                let el = document.getElementById('ok'+i);
                el.innerText = d.ok[i] ? 'ONLINE' : 'OFFLINE';
                el.className = d.ok[i] ? '' : 'off';
            }
            document.getElementById('encL').innerText = d.enc[0];
            document.getElementById('encR').innerText = d.enc[1];
        }).catch(()=>{});
    }, 200);
};

function toggleAuto() {
    let btn = document.getElementById('autoBtn');
    let willBeOn = !btn.innerText.includes('WŁ');
    let val = willBeOn ? 'auto_on' : 'auto_off';
    fetch('/cmd?dir=' + val).then(r=>r.text()).then(t => {
        updateUI(willBeOn);
        document.getElementById('st').innerText = 'Stan: ' + t;
    });
}

function cmd(c) {
    fetch('/cmd?dir=' + c).then(r => r.text()).then(t => {
        document.getElementById('st').innerText = 'Odebrano: ' + t;
    });
}

function saveCfg(id) {
    let x = document.getElementById('x'+id).value;
    let y = document.getElementById('y'+id).value;
    let a = document.getElementById('a'+id).value;
    fetch(`/api/tof_cfg?id=${id}&x=${x}&y=${y}&a=${a}`)
        .then(r=>r.text())
        .then(t => document.getElementById('st').innerText = 'Zapisano czujnik ' + id);
}
</script></body></html>
)rawliteral";

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

static esp_err_t api_tof_get_handler(httpd_req_t *req) {
    char buf[384];
    uint16_t d[3] = {0, 0, 0};
    uint8_t ok[3] = {0, 0, 0};
    ToFSensorConfig c[3] = {};

    if (robot.tof) {
        for (int i = 0; i < 3; i++) {
            d[i] = robot.tof->getDistanceMM(i);
            ok[i] = robot.tof->isOnline(i);
            c[i] = robot.tof->getSensorConfig(i);
        }
    }
    uint64_t encL = robot.motorLeft ? robot.motorLeft->encoderValue : 0;
    uint64_t encR = robot.motorRight ? robot.motorRight->encoderValue : 0;

    snprintf(buf, sizeof(buf),
        "{\"dist\":[%u,%u,%u],\"ok\":[%u,%u,%u],\"enc\":[%llu,%llu],"
        "\"cfg\":[{\"x\":%.1f,\"y\":%.1f,\"a\":%.1f},{\"x\":%.1f,\"y\":%.1f,\"a\":%.1f},{\"x\":%.1f,\"y\":%.1f,\"a\":%.1f}]}",
        d[0], d[1], d[2], ok[0], ok[1], ok[2], encL, encR,
        c[0].offsetX_mm, c[0].offsetY_mm, c[0].angle_deg,
        c[1].offsetX_mm, c[1].offsetY_mm, c[1].angle_deg,
        c[2].offsetX_mm, c[2].offsetY_mm, c[2].angle_deg);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t api_tof_cfg_handler(httpd_req_t *req) {
    char query[128], val[32];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK && robot.tof) {
        int id = 0; float x = 0, y = 0, a = 0;
        if (httpd_query_key_value(query, "id", val, sizeof(val)) == ESP_OK) id = atoi(val);
        if (httpd_query_key_value(query, "x", val, sizeof(val)) == ESP_OK) x = strtof(val, nullptr);
        if (httpd_query_key_value(query, "y", val, sizeof(val)) == ESP_OK) y = strtof(val, nullptr);
        if (httpd_query_key_value(query, "a", val, sizeof(val)) == ESP_OK) a = strtof(val, nullptr);

        robot.tof->setSensorPosition(id, x, y, a);
        httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
        return ESP_OK;
    }
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid params");
    return ESP_FAIL;
}

static esp_err_t cmd_get_handler(httpd_req_t *req) {
    char query[64];
    char dir[16];

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "dir", dir, sizeof(dir)) == ESP_OK) {
            ESP_LOGI(TAG, "Received command from web: %s", dir);

            if (strcmp(dir, "auto_on") == 0) {
                autoMode = 1;
            } else if (strcmp(dir, "auto_off") == 0) {
                autoMode = 0;
                robot.stop();
            } else if (!autoMode) {
                if (strcmp(dir, "fwd") == 0) { robot.forward(); robot.start(); }
                else if (strcmp(dir, "back") == 0) { robot.backward(); robot.start(); }
                else if (strcmp(dir, "left") == 0) { robot.rotateLeft(); robot.start(); }
                else if (strcmp(dir, "right") == 0) { robot.rotateRight(); robot.start(); }
                else if (strcmp(dir, "stop") == 0) { robot.stop(); }
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

        httpd_uri_t tof_uri = { .uri = "/api/tof", .method = HTTP_GET, .handler = api_tof_get_handler, .user_ctx = nullptr };
        httpd_register_uri_handler(server, &tof_uri);

        httpd_uri_t tof_cfg_uri = { .uri = "/api/tof_cfg", .method = HTTP_GET, .handler = api_tof_cfg_handler, .user_ctx = nullptr };
        httpd_register_uri_handler(server, &tof_cfg_uri);

        ESP_LOGI(TAG, "HTTP server creation success");
    }
}
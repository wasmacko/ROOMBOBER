#include "webserver.hpp"
#include "robot.hpp"
#include "collision.hpp"
#include "controller.hpp"
#include "odometry.hpp"
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

extern volatile uint8_t autoMode;
extern Robot robot;
extern CollisionManager* collision;
extern RobotController* controller;
extern Odometry* odometry;

static const char* TAG = "WEB_SERVER";

static const char INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html><html lang='pl'><head><meta charset='UTF-8'>
<meta name='viewport' content='width=device-width, initial-scale=1.0'>
<title>ROOMBOBER - Debug, Kolizje & Odometria</title>
<style>
body{font-family:sans-serif;text-align:center;background:#121212;color:#eee;margin:12px;}
.btn-auto{padding:11px 20px;font-size:15px;margin-bottom:10px;border:none;border-radius:8px;background:#424242;color:#fff;cursor:pointer;font-weight:bold;}
button{padding:10px 16px;font-size:14px;margin:4px;border:none;border-radius:8px;background:#2979ff;color:#fff;cursor:pointer;}
button:active{background:#1565c0;}
.btn-sm{padding:6px 12px;font-size:13px;}
.btn-red{background:#d50000;}
#st{margin:8px 0 0;color:#00e676;font-weight:bold;font-size:13px;}
.panel{background:#1e1e1e;border:1px solid #333;border-radius:10px;padding:12px;margin:10px auto;max-width:760px;}
.panel h3{margin:0 0 10px;font-size:15px;color:#90caf9;border-bottom:1px solid #333;padding-bottom:6px;}
.cards{display:flex;justify-content:space-around;flex-wrap:wrap;gap:8px;}
.card{background:#262626;padding:10px;border-radius:8px;min-width:130px;flex:1;border:2px solid transparent;}
.val{font-size:20px;font-weight:bold;color:#00e676;margin:4px 0;}
.badge{display:inline-block;padding:4px 10px;border-radius:6px;font-weight:bold;font-size:12px;background:#333;}
.ok{background:#1b5e20;color:#a5d6a7;}
.warn{background:#e65100;color:#ffe0b2;}
.crit{background:#b71c1c;color:#ffcdd2;}
.off{color:#ff5252;}
.sys-bar{display:flex;justify-content:space-around;flex-wrap:wrap;gap:8px;font-size:12px;color:#bdbdbd;}
.cfg-row{display:flex;justify-content:center;align-items:center;flex-wrap:wrap;gap:10px;margin-top:8px;font-size:13px;}
label{display:inline-flex;align-items:center;gap:4px;cursor:pointer;}
select,input[type=number]{padding:5px;background:#333;color:#fff;border:1px solid #555;border-radius:4px;text-align:center;}
input[type=number]{width:66px;}
table{width:100%;border-collapse:collapse;margin-top:6px;font-size:13px;}
th,td{padding:5px;text-align:center;}
.compass{display:inline-flex;align-items:center;justify-content:center;width:44px;height:44px;border-radius:50%;border:2px solid #90caf9;font-size:22px;font-weight:bold;color:#00e676;}
</style></head><body>

<h2>ROOMBOBER - Panel Diagnostyczny</h2>

<div class="panel">
    <div class="sys-bar">
        <span>Stan FSM: <b id="fsm" style="color:#fff;">--</b></span>
        <span>Wi-Fi: <b id="rssi">--</b> dBm</span>
        <span>RAM: <b id="ram">--</b> KB</span>
        <span>PSRAM: <b id="psram">--</b> KB</span>
        <span>Uptime: <b id="up">--</b> s</span>
    </div>
</div>

<div class="panel">
    <h3>Sterowanie (Klawiatura: W/A/S/D + Spacja = STOP)</h3>
    <div><button id='autoBtn' class='btn-auto' onclick='toggleAuto()'>Tryb Auto: Ładowanie...</button></div>
    <div>
        <button onclick="cmd('fwd')">Przód (W)</button><br>
        <button onclick="cmd('left')">Lewo (A)</button>
        <button onclick="cmd('stop')" class="btn-red">STOP (Spacja)</button>
        <button onclick="cmd('right')">Prawo (D)</button><br>
        <button onclick="cmd('back')">Tył (S)</button>
    </div>
    <div class="cfg-row" style="border-top:1px solid #333;padding-top:8px;">
        <button class="btn-sm" onclick="cmd('speeddown')">- 10%</button>
        <span>Prędkość ręczna: <b id="spd" style="color:#00e676;">--</b></span>
        <button class="btn-sm" onclick="cmd('speedup')">+ 10%</button>
    </div>
    <div id='st'>Gotowy</div>
</div>

<div class="panel">
    <h3>Warstwa 2: Odometria &amp; Kalibracja</h3>
    <div class="cards">
        <div class="card"><b>X [mm]</b><div id="odomX" class="val">0.0</div></div>
        <div class="card"><b>Y [mm]</b><div id="odomY" class="val">0.0</div></div>
        <div class="card">
            <b>Kąt θ [°]</b>
            <div style="display:flex;align-items:center;justify-content:center;gap:10px;">
                <span id="odomTh" class="val">0.0°</span>
                <div class="compass"><span id="compassArrow" style="display:inline-block;">↑</span></div>
            </div>
        </div>
        <div class="card"><b>Dystans / Prędkość</b><div id="odomDist" class="val">0.00 m</div><small id="odomVel">0 mm/s</small></div>
    </div>
    <div class="cfg-row" style="border-top:1px solid #333;padding-top:8px;">
        Średnica koła [mm]: <input id="odDia" type="number" step="0.5">
        Rozstaw kół [mm]: <input id="odBase" type="number" step="0.5">
        Imp/obr L: <input id="odTicksL" type="number" step="1">
        Imp/obr R: <input id="odTicksR" type="number" step="1">
    </div>
    <div class="cfg-row">
        <button class="btn-sm" onclick="saveOdomCfg()">Zapisz kalibrację</button>
        <button class="btn-sm btn-red" onclick="resetOdom()">Resetuj pozycję (0, 0, 0°)</button>
    </div>
</div>

<div class="panel">
    <h3>Strategia Omijania &amp; Parametry Jazdy Auto</h3>
    <div class="cfg-row">
        Strategia:
        <select id="stratSel">
            <option value="0">0: STOP_ONLY (Tylko zatrzymaj)</option>
            <option value="1">1: CLASSIC_REV_TURN (Cofnij i obróć)</option>
            <option value="2">2: SMART_OPEN_SPACE (Skręt w wolne miejsce)</option>
            <option value="3">3: SMOOTH_ARC (Łagodne omijanie łukiem)</option>
        </select>
    </div>
    <div class="cfg-row">
        Cofanie [ms]: <input id="ctrlRevMs" type="number" step="50">
        Obrót [ms]: <input id="ctrlTurnMs" type="number" step="50">
        V jazdy (0-1): <input id="ctrlCruise" type="number" step="0.05" min="0.1" max="1">
        V wolna (0-1): <input id="ctrlSlow" type="number" step="0.05" min="0.1" max="1">
        <button class="btn-sm" onclick="saveCtrlCfg()">Zapisz manewr</button>
    </div>
</div>

<div class="panel">
    <h3>Warstwa 1: Kolizje, Zderzaki &amp; Wyłączanie Czujników</h3>
    <div class="cards">
        <div class="card"><b>Zderzak Lewy</b><div id="bmpL" class="val">--</div></div>
        <div class="card"><b>Strefa Kolizji</b><div id="colSide" class="val">--</div><span id="slowBadge" class="badge">--</span></div>
        <div class="card"><b>Zderzak Prawy</b><div id="bmpR" class="val">--</div></div>
    </div>
    <div class="cfg-row" style="border-top:1px solid #333;padding-top:8px;">
        <label><input type="checkbox" id="enTof0"> ToF Lewy</label>
        <label><input type="checkbox" id="enTof1"> ToF Środek</label>
        <label><input type="checkbox" id="enTof2"> ToF Prawy</label>
        <label><input type="checkbox" id="enBmpL"> Zderzak L</label>
        <label><input type="checkbox" id="enBmpR"> Zderzak P</label>
        <label><input type="checkbox" id="enTofHard"> ToF Hard-Stop</label>
    </div>
    <div class="cfg-row">
        Hard Stop [mm]: <input id="cfgHard" type="number" step="5">
        Slow Zone [mm]: <input id="cfgSlow" type="number" step="10">
        Potwierdzenie ToF [ms]: <input id="cfgConfirm" type="number" step="20">
        <button class="btn-sm" onclick="saveColCfg()">Zapisz kolizje</button>
    </div>
</div>

<div class="panel">
    <h3>Czujniki ToF (VL53L0X) &amp; Napęd</h3>
    <div class="cards">
        <div class="card" id="card0"><b>Lewy (S0)</b><div id="d0" class="val">-- mm</div><small id="ok0">--</small></div>
        <div class="card" id="card1"><b>Środek (S1)</b><div id="d1" class="val">-- mm</div><small id="ok1">--</small></div>
        <div class="card" id="card2"><b>Prawy (S2)</b><div id="d2" class="val">-- mm</div><small id="ok2">--</small></div>
    </div>
    <div class="cards" style="margin-top:8px;">
        <div class="card"><b>Silnik Lewy</b><div>Stan: <b id="mL_st">--</b> | Imp: <b id="encL">0</b></div></div>
        <div class="card"><b>Silnik Prawy</b><div>Stan: <b id="mR_st">--</b> | Imp: <b id="encR">0</b></div></div>
    </div>
</div>

<div class="panel">
    <h3>Konfiguracja Pozycji Czujników ToF</h3>
    <table>
        <tr><th>Czujnik</th><th>X [mm]</th><th>Y [mm]</th><th>Kąt [°]</th><th>Akcja</th></tr>
        <tr><td>Lewy (0)</td><td><input id="x0" type="number" step="1"></td><td><input id="y0" type="number" step="1"></td><td><input id="a0" type="number" step="1"></td><td><button class="btn-sm" onclick="saveCfg(0)">Zapisz</button></td></tr>
        <tr><td>Środek (1)</td><td><input id="x1" type="number" step="1"></td><td><input id="y1" type="number" step="1"></td><td><input id="a1" type="number" step="1"></td><td><button class="btn-sm" onclick="saveCfg(1)">Zapisz</button></td></tr>
        <tr><td>Prawy (2)</td><td><input id="x2" type="number" step="1"></td><td><input id="y2" type="number" step="1"></td><td><input id="a2" type="number" step="1"></td><td><button class="btn-sm" onclick="saveCfg(2)">Zapisz</button></td></tr>
    </table>
</div>

<script>
const SIDE_NAMES = ['BRAK', 'LEWA', 'PRAWA', 'ŚRODEK'];
let cfgLoaded = false;

function updateAutoBtn(isOn) {
    let btn = document.getElementById('autoBtn');
    btn.style.background = isOn ? '#00e676' : '#424242';
    btn.style.color = isOn ? '#121212' : '#fff';
    btn.innerText = isOn ? 'Tryb Auto: WŁ' : 'Tryb Auto: WYŁ';
}

function pollTelemetry() {
    fetch('/api/tof').then(r=>r.json()).then(d => {
        if (!cfgLoaded) {
            for(let i=0; i<3; i++) {
                document.getElementById('x'+i).value = d.cfg[i].x;
                document.getElementById('y'+i).value = d.cfg[i].y;
                document.getElementById('a'+i).value = d.cfg[i].a;
            }
            document.getElementById('cfgHard').value = d.col.hard;
            document.getElementById('cfgSlow').value = d.col.slow;
            document.getElementById('cfgConfirm').value = d.col.conf;
            document.getElementById('enTof0').checked = d.col.en[0] === 1;
            document.getElementById('enTof1').checked = d.col.en[1] === 1;
            document.getElementById('enTof2').checked = d.col.en[2] === 1;
            document.getElementById('enBmpL').checked = d.col.en[3] === 1;
            document.getElementById('enBmpR').checked = d.col.en[4] === 1;
            document.getElementById('enTofHard').checked = d.col.en[5] === 1;

            document.getElementById('stratSel').value = d.ctrl.strat;
            document.getElementById('ctrlRevMs').value = d.ctrl.rev;
            document.getElementById('ctrlTurnMs').value = d.ctrl.trn;
            document.getElementById('ctrlCruise').value = d.ctrl.cSpd;
            document.getElementById('ctrlSlow').value = d.ctrl.sSpd;

            document.getElementById('odDia').value = d.odom.dia;
            document.getElementById('odBase').value = d.odom.base;
            document.getElementById('odTicksL').value = d.odom.tL;
            document.getElementById('odTicksR').value = d.odom.tR;
            cfgLoaded = true;
        }

        // System & FSM
        document.getElementById('fsm').innerText = d.fsm;
        document.getElementById('rssi').innerText = d.sys.rssi;
        document.getElementById('ram').innerText = d.sys.ram;
        document.getElementById('psram').innerText = d.sys.psram;
        document.getElementById('up').innerText = d.sys.up;
        document.getElementById('spd').innerText = Math.round(d.speed * 100) + '%';
        updateAutoBtn(d.auto === 1);

        // Odometria
        document.getElementById('odomX').innerText = d.odom.x.toFixed(1);
        document.getElementById('odomY').innerText = d.odom.y.toFixed(1);
        document.getElementById('odomTh').innerText = d.odom.th.toFixed(1) + '°';
        document.getElementById('compassArrow').style.transform = `rotate(${-d.odom.th}deg)`;
        document.getElementById('odomDist').innerText = (d.odom.dist / 1000.0).toFixed(2) + ' m';
        document.getElementById('odomVel').innerText = Math.round(d.odom.vel) + ' mm/s';

        // ToF
        for(let i=0; i<3; i++) {
            document.getElementById('d'+i).innerText = d.dist[i] + ' mm';
            let okEl = document.getElementById('ok'+i);
            let isEn = d.col.en[i] === 1;
            okEl.innerText = !d.ok[i] ? 'OFFLINE' : (isEn ? 'ONLINE' : 'WYŁĄCZONY');
            okEl.className = (!d.ok[i] || !isEn) ? 'off' : '';

            let card = document.getElementById('card'+i);
            if (!isEn) card.style.borderColor = '#555';
            else if (d.dist[i] > 0 && d.dist[i] < d.col.hard) card.style.borderColor = '#ff1744';
            else if (d.dist[i] > 0 && d.dist[i] < d.col.slow) card.style.borderColor = '#ff9100';
            else card.style.borderColor = 'transparent';
        }

        // Zderzaki & Kolizje
        let bL = document.getElementById('bmpL');
        bL.innerText = d.col.bL ? 'UDERZENIE!' : (d.col.en[3] ? 'OK' : 'WYŁ');
        bL.style.color = d.col.bL ? '#ff1744' : (d.col.en[3] ? '#00e676' : '#888');

        let bR = document.getElementById('bmpR');
        bR.innerText = d.col.bR ? 'UDERZENIE!' : (d.col.en[4] ? 'OK' : 'WYŁ');
        bR.style.color = d.col.bR ? '#ff1744' : (d.col.en[4] ? '#00e676' : '#888');

        let cSide = document.getElementById('colSide');
        cSide.innerText = SIDE_NAMES[d.col.side] || 'BRAK';
        cSide.style.color = d.col.side > 0 ? '#ff1744' : '#00e676';

        let sBadge = document.getElementById('slowBadge');
        if (d.col.side > 0) { sBadge.innerText = 'HARD STOP'; sBadge.className = 'badge crit'; }
        else if (d.col.isSlow) { sBadge.innerText = 'SLOW ZONE'; sBadge.className = 'badge warn'; }
        else { sBadge.innerText = 'DROGA WOLNA'; sBadge.className = 'badge ok'; }

        // Silniki & Enkodery
        document.getElementById('encL').innerText = d.enc[0];
        document.getElementById('encR').innerText = d.enc[1];
        document.getElementById('mL_st').innerText = d.mot[0] ? (d.mot[1] ? 'TYŁ' : 'PRZÓD') : 'STOP';
        document.getElementById('mR_st').innerText = d.mot[2] ? (d.mot[3] ? 'PRZÓD' : 'TYŁ') : 'STOP';
    }).catch(()=>{});
}

window.onload = function() {
    pollTelemetry();
    setInterval(pollTelemetry, 200);
};

document.addEventListener('keydown', (e) => {
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
    if (e.repeat) return;
    const k = e.key.toLowerCase();
    if (k === 'w') cmd('fwd');
    else if (k === 's') cmd('back');
    else if (k === 'a') cmd('left');
    else if (k === 'd') cmd('right');
    else if (k === ' ') { e.preventDefault(); cmd('stop'); }
});

function toggleAuto() {
    let btn = document.getElementById('autoBtn');
    let willBeOn = !btn.innerText.includes('WŁ');
    fetch('/cmd?dir=' + (willBeOn ? 'auto_on' : 'auto_off')).then(r=>r.text()).then(t => {
        updateAutoBtn(willBeOn);
        document.getElementById('st').innerText = 'Tryb: ' + t;
    });
}

function cmd(c) {
    fetch('/cmd?dir=' + c).then(r => r.text()).then(t => {
        document.getElementById('st').innerText = 'Komenda: ' + t;
    });
}

function saveCfg(id) {
    let x = document.getElementById('x'+id).value;
    let y = document.getElementById('y'+id).value;
    let a = document.getElementById('a'+id).value;
    fetch(`/api/tof_cfg?id=${id}&x=${x}&y=${y}&a=${a}`)
        .then(() => document.getElementById('st').innerText = 'Zapisano pozycję ToF #' + id);
}

function saveColCfg() {
    let h  = document.getElementById('cfgHard').value;
    let s  = document.getElementById('cfgSlow').value;
    let c  = document.getElementById('cfgConfirm').value;
    let t0 = document.getElementById('enTof0').checked ? 1 : 0;
    let t1 = document.getElementById('enTof1').checked ? 1 : 0;
    let t2 = document.getElementById('enTof2').checked ? 1 : 0;
    let bL = document.getElementById('enBmpL').checked ? 1 : 0;
    let bR = document.getElementById('enBmpR').checked ? 1 : 0;
    let th = document.getElementById('enTofHard').checked ? 1 : 0;
    fetch(`/api/col_cfg?hard=${h}&slow=${s}&conf=${c}&t0=${t0}&t1=${t1}&t2=${t2}&bL=${bL}&bR=${bR}&th=${th}`)
        .then(() => document.getElementById('st').innerText = 'Zapisano ustawienia kolizji i czujników');
}

function saveCtrlCfg() {
    let st = document.getElementById('stratSel').value;
    let rv = document.getElementById('ctrlRevMs').value;
    let tr = document.getElementById('ctrlTurnMs').value;
    let cs = document.getElementById('ctrlCruise').value;
    let ss = document.getElementById('ctrlSlow').value;
    fetch(`/api/ctrl_cfg?strat=${st}&rev=${rv}&trn=${tr}&cSpd=${cs}&sSpd=${ss}`)
        .then(() => document.getElementById('st').innerText = 'Zapisano strategię omijania');
}

function saveOdomCfg() {
    let d  = document.getElementById('odDia').value;
    let b  = document.getElementById('odBase').value;
    let tL = document.getElementById('odTicksL').value;
    let tR = document.getElementById('odTicksR').value;
    fetch(`/api/odom_cfg?dia=${d}&base=${b}&tL=${tL}&tR=${tR}`)
        .then(() => document.getElementById('st').innerText = 'Zapisano kalibrację odometrii');
}

function resetOdom() {
    fetch('/api/odom_reset')
        .then(() => document.getElementById('st').innerText = 'Zresetowano odometrię do (0, 0, 0°)');
}
</script></body></html>
)rawliteral";

static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_get_handler(httpd_req_t *req) {
    const char* resp = autoMode ? "on" : "off";
    return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t api_tof_get_handler(httpd_req_t *req) {
    static char buf[1280];
    uint16_t d[3] = {0, 0, 0};
    uint8_t ok[3] = {0, 0, 0};
    ToFSensorConfig c[3] = {};

    if (robot.tof) {
        for (int i = 0; i < 3; i++) {
            d[i]  = robot.tof->getDistanceMM(i);
            ok[i] = robot.tof->isOnline(i);
            c[i]  = robot.tof->getSensorConfig(i);
        }
    }

    long long encL = robot.motorLeft  ? (long long)robot.motorLeft->getSignedEncoderValue()  : 0;
    long long encR = robot.motorRight ? (long long)robot.motorRight->getSignedEncoderValue() : 0;

    uint8_t mL_on  = robot.motorLeft  ? robot.motorLeft->isOn()       : 0;
    uint8_t mL_dir = robot.motorLeft  ? robot.motorLeft->direction()  : 0;
    uint8_t mR_on  = robot.motorRight ? robot.motorRight->isOn()      : 0;
    uint8_t mR_dir = robot.motorRight ? robot.motorRight->direction() : 0;

    uint8_t bL = 0, bR = 0, cSide = 0, isSlow = 0;
    uint16_t hardMm = 70, slowMm = 220;
    uint32_t confMs = 120;
    uint8_t en[6] = {1, 1, 1, 1, 1, 1};
    if (collision) {
        bL     = collision->isLeftBumperHit() ? 1 : 0;
        bR     = collision->isRightBumperHit() ? 1 : 0;
        cSide  = static_cast<uint8_t>(collision->getCollisionSide());
        isSlow = collision->isSlowZone() ? 1 : 0;
        hardMm = collision->config.hardStopDist_mm;
        slowMm = collision->config.slowDownDist_mm;
        confMs = collision->config.tofConfirm_ms;
        en[0]  = collision->config.tofEnabled[0] ? 1 : 0;
        en[1]  = collision->config.tofEnabled[1] ? 1 : 0;
        en[2]  = collision->config.tofEnabled[2] ? 1 : 0;
        en[3]  = collision->config.bumperLeftEnabled ? 1 : 0;
        en[4]  = collision->config.bumperRightEnabled ? 1 : 0;
        en[5]  = collision->config.tofHardStopEnabled ? 1 : 0;
    }

    const char* fsmState = controller ? controller->getStateName() : "N/A";
    uint8_t strat = controller ? static_cast<uint8_t>(controller->strategy) : 0;
    uint32_t revMs = controller ? controller->reverseDuration_ms : 350;
    uint32_t trnMs = controller ? controller->turnDuration_ms : 450;
    double cSpd = controller ? controller->cruiseSpeed : 0.55;
    double sSpd = controller ? controller->slowSpeed : 0.35;

    RobotPose pose{};
    OdometryConfig oCfg{};
    float thDeg = 0.0f;
    if (odometry) {
        pose  = odometry->getPose();
        oCfg  = odometry->getConfig();
        thDeg = odometry->getThetaDeg();
    }

    wifi_ap_record_t ap_info;
    int rssi = (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) ? ap_info.rssi : 0;
    uint32_t freeRamKb   = esp_get_free_heap_size() / 1024;
    uint32_t freePsramKb = heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024;
    uint32_t uptimeSec   = (uint32_t)(esp_timer_get_time() / 1000000ULL);

    snprintf(buf, sizeof(buf),
        "{\"auto\":%u,\"fsm\":\"%s\",\"speed\":%.2f,"
        "\"dist\":[%u,%u,%u],\"ok\":[%u,%u,%u],"
        "\"enc\":[%lld,%lld],\"mot\":[%u,%u,%u,%u],"
        "\"col\":{\"bL\":%u,\"bR\":%u,\"side\":%u,\"isSlow\":%u,\"hard\":%u,\"slow\":%u,\"conf\":%lu,\"en\":[%u,%u,%u,%u,%u,%u]},"
        "\"ctrl\":{\"strat\":%u,\"rev\":%lu,\"trn\":%lu,\"cSpd\":%.2f,\"sSpd\":%.2f},"
        "\"odom\":{\"x\":%.1f,\"y\":%.1f,\"th\":%.1f,\"dist\":%.1f,\"vel\":%.1f,\"dia\":%.1f,\"base\":%.1f,\"tL\":%.1f,\"tR\":%.1f},"
        "\"sys\":{\"rssi\":%d,\"ram\":%lu,\"psram\":%lu,\"up\":%lu},"
        "\"cfg\":[{\"x\":%.1f,\"y\":%.1f,\"a\":%.1f},{\"x\":%.1f,\"y\":%.1f,\"a\":%.1f},{\"x\":%.1f,\"y\":%.1f,\"a\":%.1f}]}",
        autoMode, fsmState, robot.speed,
        d[0], d[1], d[2], ok[0], ok[1], ok[2],
        encL, encR, mL_on, mL_dir, mR_on, mR_dir,
        bL, bR, cSide, isSlow, hardMm, slowMm, (unsigned long)confMs,
        en[0], en[1], en[2], en[3], en[4], en[5],
        strat, (unsigned long)revMs, (unsigned long)trnMs, cSpd, sSpd,
        pose.x_mm, pose.y_mm, thDeg, pose.totalDist_mm, pose.linearVel_mms,
        oCfg.wheelDiameter_mm, oCfg.wheelBase_mm, oCfg.ticksPerRevLeft, oCfg.ticksPerRevRight,
        rssi, (unsigned long)freeRamKb, (unsigned long)freePsramKb, (unsigned long)uptimeSec,
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
        return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid params");
    return ESP_FAIL;
}

static esp_err_t api_col_cfg_handler(httpd_req_t *req) {
    char query[160], val[16];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK && collision) {
        if (httpd_query_key_value(query, "hard", val, sizeof(val)) == ESP_OK) collision->config.hardStopDist_mm = (uint16_t)atoi(val);
        if (httpd_query_key_value(query, "slow", val, sizeof(val)) == ESP_OK) collision->config.slowDownDist_mm = (uint16_t)atoi(val);
        if (httpd_query_key_value(query, "conf", val, sizeof(val)) == ESP_OK) collision->config.tofConfirm_ms   = (uint32_t)atoi(val);
        if (httpd_query_key_value(query, "t0", val, sizeof(val)) == ESP_OK)   collision->config.tofEnabled[0]   = (atoi(val) != 0);
        if (httpd_query_key_value(query, "t1", val, sizeof(val)) == ESP_OK)   collision->config.tofEnabled[1]   = (atoi(val) != 0);
        if (httpd_query_key_value(query, "t2", val, sizeof(val)) == ESP_OK)   collision->config.tofEnabled[2]   = (atoi(val) != 0);
        if (httpd_query_key_value(query, "bL", val, sizeof(val)) == ESP_OK)   collision->config.bumperLeftEnabled  = (atoi(val) != 0);
        if (httpd_query_key_value(query, "bR", val, sizeof(val)) == ESP_OK)   collision->config.bumperRightEnabled = (atoi(val) != 0);
        if (httpd_query_key_value(query, "th", val, sizeof(val)) == ESP_OK)   collision->config.tofHardStopEnabled = (atoi(val) != 0);
        return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid params");
    return ESP_FAIL;
}

static esp_err_t api_ctrl_cfg_handler(httpd_req_t *req) {
    char query[128], val[16];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK && controller) {
        if (httpd_query_key_value(query, "strat", val, sizeof(val)) == ESP_OK) {
            int s = atoi(val);
            if (s >= 0 && s <= 3) controller->strategy = static_cast<AvoidStrategy>(s);
        }
        if (httpd_query_key_value(query, "rev",  val, sizeof(val)) == ESP_OK) controller->reverseDuration_ms = (uint32_t)atoi(val);
        if (httpd_query_key_value(query, "trn",  val, sizeof(val)) == ESP_OK) controller->turnDuration_ms    = (uint32_t)atoi(val);
        if (httpd_query_key_value(query, "cSpd", val, sizeof(val)) == ESP_OK) controller->cruiseSpeed        = strtod(val, nullptr);
        if (httpd_query_key_value(query, "sSpd", val, sizeof(val)) == ESP_OK) controller->slowSpeed          = strtod(val, nullptr);
        return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid params");
    return ESP_FAIL;
}

static esp_err_t api_odom_cfg_handler(httpd_req_t *req) {
    char query[128], val[16];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK && odometry) {
        OdometryConfig cfg = odometry->getConfig();
        if (httpd_query_key_value(query, "dia",  val, sizeof(val)) == ESP_OK) cfg.wheelDiameter_mm = strtof(val, nullptr);
        if (httpd_query_key_value(query, "base", val, sizeof(val)) == ESP_OK) cfg.wheelBase_mm     = strtof(val, nullptr);
        if (httpd_query_key_value(query, "tL",   val, sizeof(val)) == ESP_OK) cfg.ticksPerRevLeft  = strtof(val, nullptr);
        if (httpd_query_key_value(query, "tR",   val, sizeof(val)) == ESP_OK) cfg.ticksPerRevRight = strtof(val, nullptr);
        odometry->setConfig(cfg);
        return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
    }
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid params");
    return ESP_FAIL;
}

static esp_err_t api_odom_reset_handler(httpd_req_t *req) {
    if (odometry) odometry->reset(true);
    return httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
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
                if (strcmp(dir, "fwd") == 0)        { robot.uploadSpeed(); robot.forward(); robot.start(); }
                else if (strcmp(dir, "back") == 0)  { robot.uploadSpeed(); robot.backward(); robot.start(); }
                else if (strcmp(dir, "left") == 0)  { robot.uploadSpeed(); robot.rotateLeft(); robot.start(); }
                else if (strcmp(dir, "right") == 0) { robot.uploadSpeed(); robot.rotateRight(); robot.start(); }
                else if (strcmp(dir, "stop") == 0)  { robot.stop(); }
                else if (strcmp(dir, "speedup") == 0) {
                    robot.speed += 0.1;
                    if (robot.speed > 1.0) robot.speed = 1.0;
                    robot.uploadSpeed();
                } else if (strcmp(dir, "speeddown") == 0) {
                    robot.speed -= 0.1;
                    if (robot.speed < 0.0) robot.speed = 0.0;
                    robot.uploadSpeed();
                }
            }
            return httpd_resp_send(req, dir, HTTPD_RESP_USE_STRLEN);
        }
    }
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No dir parameter.");
    return ESP_FAIL;
}

void WebServer::start() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 16;
    config.stack_size       = 8192;
    httpd_handle_t server = nullptr;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t uris[] = {
            { .uri = "/",               .method = HTTP_GET, .handler = root_get_handler,       .user_ctx = nullptr },
            { .uri = "/status",         .method = HTTP_GET, .handler = status_get_handler,     .user_ctx = nullptr },
            { .uri = "/cmd",            .method = HTTP_GET, .handler = cmd_get_handler,        .user_ctx = nullptr },
            { .uri = "/api/tof",        .method = HTTP_GET, .handler = api_tof_get_handler,    .user_ctx = nullptr },
            { .uri = "/api/tof_cfg",    .method = HTTP_GET, .handler = api_tof_cfg_handler,    .user_ctx = nullptr },
            { .uri = "/api/col_cfg",    .method = HTTP_GET, .handler = api_col_cfg_handler,    .user_ctx = nullptr },
            { .uri = "/api/ctrl_cfg",   .method = HTTP_GET, .handler = api_ctrl_cfg_handler,   .user_ctx = nullptr },
            { .uri = "/api/odom_cfg",   .method = HTTP_GET, .handler = api_odom_cfg_handler,   .user_ctx = nullptr },
            { .uri = "/api/odom_reset", .method = HTTP_GET, .handler = api_odom_reset_handler, .user_ctx = nullptr },
        };
        for (const auto &u : uris) {
            httpd_register_uri_handler(server, &u);
        }
        ESP_LOGI(TAG, "HTTP server started with %d handlers", (int)(sizeof(uris)/sizeof(uris[0])));
    }
}
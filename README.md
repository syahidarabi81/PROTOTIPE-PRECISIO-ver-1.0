/*
=====================================================================
PENYEMAI BENIH HIDROPONIK OTOMATIS - MANIPULATOR KARTESIAN (v3)
Mekanisme ELEKTROSTATIK: 13 elektroda sekrup M3 LEPAS-PASANG + 1 modul
tegangan tinggi + 1 relay (masukan 24 V modul HV). Tanpa pompa vakum.
- Tiap posisi memiliki soket kuningan berulir M3; kabel HV dari bus masuk
ke terminal soket. Elektroda (stud M3 + kenop) dipasang/dilepas dari
bawah dengan tangan. Sekrup yang tidak dipakai cukup dilepas.
- Wadah benih digetarkan (motor getar DC + peredam karet) untuk mengaduk
dan meratakan benih setiap kali benih akan diambil.
- Panel box + HMI berada di alas putih yang sama dengan robot (luar rangka),
di kanan operator: tombol -> LCD 20x4 -> keypad 4x4 (keypad paling dekat).
- Nampan dijepit penjepit X-Y adjustable; jangkauan baris sekrup dibatasi
X_MAKS agar sesuai stopper rel penjepit.
---------------------------------------------------------------------
Papan : Arduino Mega 2560
Pustaka : AccelStepper, Keypad, LiquidCrystal_I2C, Wire, EEPROM
HMI : LCD 20x4 I2C + keypad 4x4 + tombol darurat + lampu "HV ON"
Menu : 1 Mulai Penyemaian 2 Parameter Aktif
3 Sekrup Terpasang 4 Parameter Penyemai
5 Kalibrasi Stepper 6 Kalibrasi Elektrostatis
7 Penggetar Wadah 8 Pengaturan Sistem
Keypad : A = naik / tambah B = turun / kurang
C = kembali / batal D = OK / pilih
1..9 = pilih langsung item ke-n
* = titik desimal # = tanda minus (saat isi angka)
---------------------------------------------------------------------
Koordinat:
X = posisi gantri (mm) dari titik home (limit switch X), 0..X_MAKS.
h = tinggi UJUNG sekrup di atas permukaan rockwool / pelat cekungan
wadah benih (keduanya Z = 61 mm pada model CAD).
Negatif = masuk ke lubang tanam. Home Z = posisi paling atas.
Menu 3 "Sekrup Terpasang" hanya CATATAN posisi soket yang diisi sekrup
(untuk menghitung benih per nampan & pengingat pola). Semua soket
tersambung ke HV yang sama; ON/OFF dilakukan dengan memasang/melepas sekrup.
Nilai bawaan diambil dari model FreeCAD "Penyemai_Elektrostatik_13Sekrup_v3.3".
v3.1: teks peringatan HV diperbarui (tanpa papan relay); pin tidak berubah.
=====================================================================
*/
#include <AccelStepper.h>
#include <Keypad.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#define FW_VERSI "v3.1"
typedef void (*FmtItem)(uint8_t i, char* out); // pemformat item menu
// ------------------------------- PIN --------------------------------
const uint8_t PIN_LIM_X = 2; // limit switch X (NO ke GND, aktif LOW)
const uint8_t PIN_LIM_Z = 3; // limit switch Z atas (NO ke GND, aktif LOW)
const uint8_t PIN_ESTOP = 18; // tombol henti darurat (NC ke GND; ditekan = HIGH)
const uint8_t PIN_X_STEP = 4, PIN_X_DIR = 5;
const uint8_t PIN_Z_STEP = 6, PIN_Z_DIR = 7;
const uint8_t PIN_EN = 8; // ENABLE kedua A4988 (aktif LOW)
const uint8_t PIN_HV_CTRL = 9; // PWM 490 Hz -> RC 10k/10uF -> CTRL modul HV (0-5 V
= 0..-10 kV)
const uint8_t PIN_HV_SDN = 10; // SDN modul HV (LOW = mati)
const uint8_t PIN_RLY_HV = 11; // relay masukan 24 V modul HV (pengaman kedua)
const uint8_t PIN_GETAR = 12; // PWM -> MOSFET IRLZ44N -> motor getar 12 V wadah
benih

const uint8_t PIN_LAMPU_HV = 13; // lampu indikator "HV ON" di panel HMI (lewat
transistor)
const uint8_t N_SEKRUP = 13; // jumlah soket elektroda
const uint16_t MASK_SEMUA = (1u << N_SEKRUP) - 1; // 0x1FFF
#define RELAY_AKTIF_LOW 1 // modul relay 1 kanal (satu-satunya relay: masukan
modul HV) aktif LOW
const float KV_MAKS = 10.0; // batas modul HV (0..-10 kV); medan dicek terpisah
(< 2,5 kV/mm)
const float X_MAKS = 538.0; // X gantri maks dari home (X_g 638): gantri belum
menyentuh poros sinkron
const float H_AMAN_MIN = 15.0; // tinggi aman minimum: ujung sekrup melewati kenop
penjepit nampan
// ------------------------------ KEYPAD ------------------------------
const byte KP_ROWS = 4, KP_COLS = 4;
char kpKeys[KP_ROWS][KP_COLS] = {
{'1', '2', '3', 'A'},
{'4', '5', '6', 'B'},
{'7', '8', '9', 'C'},
{'*', '0', '#', 'D'}
};
byte kpRowPins[KP_ROWS] = {22, 24, 26, 28};
byte kpColPins[KP_COLS] = {30, 32, 34, 36};
Keypad keypad = Keypad(makeKeymap(kpKeys), kpRowPins, kpColPins, KP_ROWS, KP_COLS);
LiquidCrystal_I2C lcd(0x27, 20, 4);
AccelStepper stX(AccelStepper::DRIVER, PIN_X_STEP, PIN_X_DIR);
AccelStepper stZ(AccelStepper::DRIVER, PIN_Z_STEP, PIN_Z_DIR);
// ---------------------------- PARAMETER -----------------------------
enum : uint8_t { BENIH_BAYAM = 0, BENIH_PAKCOY = 1, BENIH_KUSTOM = 2, N_BENIH = 3 };
const char* const NAMA_BENIH[N_BENIH] = {"Bayam", "Pakcoy", "Kustom"};
struct ProfilBenih {
float hAmbil; // tinggi ujung sekrup saat ambil (mm di atas pelat cekungan)
float hTanam; // tinggi ujung saat tanam (mm, negatif = masuk lubang)
float kV; // tegangan elektroda (kV, nilai mutlak, maks KV_MAKS)
uint16_t tTahan; // ms HV menyala di posisi ambil sebelum naik
uint16_t tLepas; // ms tunggu setelah HV dimatikan di atas lubang
uint8_t getarPwm; // intensitas penggetar (%)
uint16_t tGetar; // lama penggetaran sebelum ambil (ms)
uint16_t tTenang; // jeda setelah getar berhenti agar benih diam di cekungan (ms)
};
struct Parameter {
uint16_t magic;
uint8_t benih; // jenis benih aktif
ProfilBenih prof[N_BENIH];
uint16_t maskSekrup; // bit i = soket ke-(i+1) terisi sekrup (catatan)
uint8_t getarSaatJalan; // 1 = mulai getar saat gantri berjalan ke wadah (hemat
waktu)
uint8_t getarSaatAmbil; // 1 = penggetar tetap menyala saat HV ON di posisi ambil
float jarakBaris; // jarak antar baris tray (mm)
uint8_t barisAwal; // 1..17
uint8_t jumlahBaris; // jumlah baris semai
float vX, vZ; // kecepatan (mm/s)
float aX, aZ; // percepatan (mm/s^2)
float langkahX, langkahZ; // langkah/mm
float xAmbil; // X gantri saat barisan sekrup di atas cekungan wadah
float xBaris1; // X gantri saat barisan sekrup di atas baris 1 nampan
float hHome; // tinggi ujung sekrup saat Z di home atas (mm)
float hAman; // tinggi aman saat gantri bergerak (mm)
uint32_t totalNampan; // penghitung nampan selesai
};
const uint16_t MAGIC = 0xB3E6; // berbeda dari v1/v2 -> EEPROM lama otomatis direset
Parameter P;
void setDefault() {
P.magic = MAGIC;
P.benih = BENIH_BAYAM;
// hAmbil hTanam kV tTahan tLepas pwm tGetar tTenang
P.prof[BENIH_BAYAM] = {3.0, 2.0, 6.0, 500, 500, 60, 1500, 400};

P.prof[BENIH_PAKCOY] = {2.0, 1.0, 5.0, 500, 500, 80, 1500, 300};
P.prof[BENIH_KUSTOM] = {2.5, 1.5, 5.5, 500, 500, 70, 1500, 400};
P.maskSekrup = MASK_SEMUA;
P.getarSaatJalan = 1;
P.getarSaatAmbil = 0;
P.jarakBaris = 27.0;
P.barisAwal = 1;
P.jumlahBaris = 17;
P.vX = 100.0; P.vZ = 30.0;
P.aX = 1000.0; P.aZ = 300.0;
P.langkahX = 40.0; // GT2 20T (40 mm/put), 1/8 step
P.langkahZ = 50.93; // pinion m1 z20 (62,83 mm/put), 1/16 step
P.xAmbil = 8.5; // home switch X di X_g = 100 mm (CAD): sekrup di atas
cekungan (X = 83) pada X_g 108,5
P.xBaris1 = 86.5; // pose CAD: penjepit kiri X 125..145, baris 1 di X = 161 ->
X_g 186,5 (kalibrasi di menu 5)
P.hHome = 74.0; // ujung sekrup Z = 135 mm, permukaan rockwool & pelat
cekungan Z = 61 mm
P.hAman = 20.0;
P.totalNampan = 0;
}
void simpanParameter() { EEPROM.put(0, P); }
void muatParameter() {
EEPROM.get(0, P);
if (P.magic != MAGIC) { setDefault(); simpanParameter(); }
}
ProfilBenih& PB() { return P.prof[P.benih]; }
// ---------------------------- STATUS --------------------------------
volatile bool estop = false;
bool sudahHome = false;
bool hvSiaga = false; // relay masukan 24 V modul HV tertutup
bool hvAktif = false; // SDN HIGH (tegangan tinggi keluar)
bool getarAktif = false;
// ---------------------------- AKTUATOR ------------------------------
void relay(uint8_t pin, bool on) {
#if RELAY_AKTIF_LOW
digitalWrite(pin, on ? LOW : HIGH);
#else
digitalWrite(pin, on ? HIGH : LOW);
#endif
}
void hvMati() { // SDN LOW: keluaran HV turun, bleeder 100 MOhm
membuang muatan
analogWrite(PIN_HV_CTRL, 0);
digitalWrite(PIN_HV_SDN, LOW);
hvAktif = false;
}
void hvPutus() { // matikan HV dan putus juga masukan 24 V modul
hvMati();
relay(PIN_RLY_HV, false);
digitalWrite(PIN_LAMPU_HV, LOW);
hvSiaga = false;
}
void hvNyala(float kV) {
if (estop) return;
kV = constrain(kV, 0.0, KV_MAKS);
if (!hvSiaga) { // relay masukan cukup ditutup sekali per sesi
relay(PIN_RLY_HV, true);
digitalWrite(PIN_LAMPU_HV, HIGH);
hvSiaga = true;
delay(30);
}
analogWrite(PIN_HV_CTRL, (uint8_t)lround(kV / 10.0 * 255.0)); // 0..5 V -> 0..-10 kV
digitalWrite(PIN_HV_SDN, HIGH);
hvAktif = true;
}
void getarMati() { analogWrite(PIN_GETAR, 0); getarAktif = false; }
void getar(uint8_t persen) {
if (estop) return;
persen = constrain(persen, 0, 100);

if (persen == 0) { getarMati(); return; }
if (!getarAktif && persen < 60) { analogWrite(PIN_GETAR, 255); delay(60); } //
hentakan awal agar motor mulai berputar
analogWrite(PIN_GETAR, (uint8_t)((uint16_t)persen * 255 / 100));
getarAktif = true;
}
void semuaAman() { // matikan seluruh aktuator berbahaya
getarMati();
hvPutus();
}
void isrEstop() { // prioritas tertinggi: putus HV, penggetar, motor
estop = true;
digitalWrite(PIN_EN, HIGH);
analogWrite(PIN_HV_CTRL, 0);
digitalWrite(PIN_HV_SDN, LOW);
relay(PIN_RLY_HV, false);
digitalWrite(PIN_LAMPU_HV, LOW);
analogWrite(PIN_GETAR, 0);
}
// ----------------------------- SEKRUP -------------------------------
bool sekrupAktif(uint8_t i) { return (P.maskSekrup >> i) & 1; }
uint8_t jumlahSekrup(uint16_t m) {
uint8_t n = 0;
for (uint8_t i = 0; i < N_SEKRUP; i++) n += (m >> i) & 1;
return n;
}
void polaSekrup(uint16_t m, char* out) { // 13 karakter: kotak penuh = terpasang,
kotak kosong = kosong
for (uint8_t i = 0; i < N_SEKRUP; i++) out[i] = ((m >> i) & 1) ? '\x01' : '\x02';
out[N_SEKRUP] = 0;
}
// ------------------------------ LCD ---------------------------------
char buf[24];
byte chrPenuh[8] = {0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x00};
byte chrKosong[8] = {0x00, 0x1F, 0x11, 0x11, 0x11, 0x11, 0x1F, 0x00};
void baris(uint8_t r, const char* s) {
lcd.setCursor(0, r);
uint8_t n = 0;
while (s[n] && n < 20) { lcd.write(s[n]); n++; }
while (n++ < 20) lcd.write(' ');
}
void baris(uint8_t r, const __FlashStringHelper* fs) {
char t[21]; strncpy_P(t, (PGM_P)fs, 20); t[20] = 0; baris(r, t);
}
char* fmtF(char* out, float v, uint8_t prec) { // AVR tidak mendukung %f
dtostrf(v, 0, prec, out);
return out;
}
void layarPesan(const __FlashStringHelper* a, const __FlashStringHelper* b, uint16_t ms)
{
lcd.clear(); baris(1, a); baris(2, b); delay(ms);
}
// ---------------------------- INPUT ---------------------------------
char tungguTombol() {
char k;
while (!(k = keypad.getKey())) { if (estop) return 'C'; }
return k;
}
// Tunggu ms milidetik; false bila henti darurat atau operator menekan C
bool tunggu(uint32_t ms) {
uint32_t t0 = millis();
while (millis() - t0 < ms) {
if (estop) return false;
if (keypad.getKey() == 'C') return false;
}
return true;
}
// Isi angka melalui keypad

bool isiAngka(const char* judul, float& nilai, float minV, float maxV, uint8_t prec) {
char in[10] = ""; uint8_t n = 0; char t[12];
lcd.clear();
baris(0, judul);
snprintf(buf, sizeof(buf), "Kini: %s", fmtF(t, nilai, prec)); baris(1, buf);
baris(3, F("*=. #=- A=hps D=OK"));
while (true) {
snprintf(buf, sizeof(buf), "Baru: %s_", in); baris(2, buf);
char k = tungguTombol();
if (k >= '0' && k <= '9' && n < 8) { in[n++] = k; in[n] = 0; }
else if (k == '*' && n < 8 && !strchr(in, '.')) { in[n++] = '.'; in[n] = 0; }
else if (k == '#' && n == 0) { in[n++] = '-'; in[n] = 0; }
else if (k == 'A' && n > 0) { in[--n] = 0; }
else if (k == 'C') return false;
else if (k == 'D') {
if (n == 0) return false;
float v = atof(in);
if (v < minV || v > maxV) {
char t2[12];
snprintf(buf, sizeof(buf), "Rentang %s..%s", fmtF(t, minV, prec), fmtF(t2, maxV,
prec));
baris(2, buf); delay(900);
n = 0; in[0] = 0; continue;
}
nilai = v; return true;
}
}
}
bool isiInt(const char* judul, uint16_t& nilai, uint16_t minV, uint16_t maxV) {
float f = nilai;
if (!isiAngka(judul, f, minV, maxV, 0)) return false;
nilai = (uint16_t)lround(f); return true;
}
bool isiByte(const char* judul, uint8_t& nilai, uint8_t minV, uint8_t maxV) {
float f = nilai;
if (!isiAngka(judul, f, minV, maxV, 0)) return false;
nilai = (uint8_t)lround(f); return true;
}
// Menu daftar generik: fmt(i, out) mengisi teks item ke-i
int8_t pilihMenu(const char* judul, uint8_t n, FmtItem fmt, uint8_t awal = 0) {
uint8_t cur = awal < n ? awal : 0, top = 0;
char t[22];
if (cur >= 3) top = cur - 2;
while (true) {
if (cur < top) top = cur;
if (cur >= top + 3) top = cur - 2;
baris(0, judul);
for (uint8_t r = 0; r < 3; r++) {
uint8_t i = top + r;
if (i < n) { fmt(i, t); snprintf(buf, sizeof(buf), "%c%s", i == cur ? '>' : ' ',
t); baris(r + 1, buf); }
else baris(r + 1, "");
}
char k = tungguTombol();
if (k == 'A') cur = (cur == 0) ? n - 1 : cur - 1;
else if (k == 'B') cur = (cur + 1) % n;
else if (k == 'D') return cur;
else if (k == 'C') return -1;
else if (k >= '1' && k <= '9' && (uint8_t)(k - '1') < n) return k - '1';
}
}
// ---------------------------- GERAK ---------------------------------
void terapkanKecepatan() {
stX.setMaxSpeed(P.vX * P.langkahX); stX.setAcceleration(P.aX * P.langkahX);
stZ.setMaxSpeed(P.vZ * P.langkahZ); stZ.setAcceleration(P.aZ * P.langkahZ);
}
long zLangkah(float h) { return lround((h - P.hHome) * P.langkahZ); } // home atas = 0,
turun = negatif
float zSekarang() { return P.hHome + stZ.currentPosition() / P.langkahZ; }
float xSekarang() { return stX.currentPosition() / P.langkahX; }
// Jalankan kedua motor sampai target; return false bila henti darurat / dibatalkan

bool jalankan(bool bolehBatal = true) {
digitalWrite(PIN_EN, LOW);
uint32_t tk = 0;
while (stX.distanceToGo() != 0 || stZ.distanceToGo() != 0) {
if (estop) return false;
stX.run(); stZ.run();
if (bolehBatal && millis() - tk > 60) {
tk = millis();
if (keypad.getKey() == 'C') { stX.stop(); stZ.stop(); stX.runToPosition();
stZ.runToPosition(); return false; }
}
}
return !estop;
}
bool keX(float mm) { mm = constrain(mm, 0.0, X_MAKS); stX.moveTo(lround(mm *
P.langkahX)); return jalankan(); }
float xBaris(uint8_t b);
// batas bawah lunak: ujung sekrup boleh masuk (h < 0) hanya tepat di atas baris lubang
tanam,
// di tempat lain (mis. di atas wadah benih / dinding nampan) minimal 0,5 mm di atas
permukaan
float hMinimum(float x) {
for (uint8_t b = 1; b <= 17; b++) if (fabs(x - xBaris(b)) < 5.0) return -10.0;
return 0.5;
}
bool keZ(float h) { h = constrain(h, hMinimum(xSekarang()), P.hHome);
stZ.moveTo(zLangkah(h)); return jalankan(); }
bool homingSumbu(AccelStepper& s, uint8_t pinLim, int arah, float langkahPerMm, float
vCari) {
// cari cepat -> mundur 3 mm -> cari lambat
s.setMaxSpeed(vCari * langkahPerMm); s.setAcceleration(400 * langkahPerMm);
digitalWrite(PIN_EN, LOW);
for (uint8_t fase = 0; fase < 2; fase++) {
s.setSpeed(arah * (fase == 0 ? vCari : vCari / 4) * langkahPerMm);
uint32_t t0 = millis();
while (digitalRead(pinLim) == HIGH) {
if (estop || millis() - t0 > 30000UL) return false;
s.runSpeed();
}
s.setCurrentPosition(0);
s.moveTo(lround(-arah * 3.0 * langkahPerMm));
while (s.distanceToGo() != 0) { if (estop) return false; s.run(); }
}
s.setCurrentPosition(0);
return true;
}
bool homing() {
lcd.clear(); baris(0, F("HOMING...")); baris(1, F("Sumbu Z ke atas"));
semuaAman();
if (!homingSumbu(stZ, PIN_LIM_Z, +1, P.langkahZ, 15)) return false;
stZ.setCurrentPosition(zLangkah(P.hHome - 3.0)); // posisi Z = home - 3 mm (mundur
dari switch)
baris(1, F("Sumbu X ke home"));
if (!homingSumbu(stX, PIN_LIM_X, -1, P.langkahX, 40)) return false;
stX.setCurrentPosition(lround(3.0 * P.langkahX));
terapkanKecepatan();
sudahHome = true;
return true;
}
// X gantri agar barisan sekrup berada di atas baris b (1-based)
float xBaris(uint8_t b) { return P.xBaris1 + (b - 1) * P.jarakBaris; }
// ------------------------ PENANGANAN DARURAT ------------------------
void tanganiDarurat() {
semuaAman();
digitalWrite(PIN_EN, HIGH);
sudahHome = false;
lcd.clear();
baris(0, F("!! HENTI DARURAT !!"));
baris(1, F("HV, penggetar &"));
baris(2, F("motor diputus."));
baris(3, F("Lepas tombol + D"));

while (digitalRead(PIN_ESTOP) == HIGH) delay(20);
while (keypad.getKey() != 'D') delay(5);
estop = false;
}
// ------------------------- SIKLUS SEMAI -----------------------------
// Ambil: getarkan wadah -> benih tenang di cekungan -> turun -> HV ON -> naik
bool ambilBenih() {
ProfilBenih& pb = PB();
if (!keZ(P.hAman)) return false;
uint32_t tGetar0 = 0;
if (P.getarSaatJalan) { getar(pb.getarPwm); tGetar0 = millis(); } // aduk benih
selama gantri berjalan
if (!keX(P.xAmbil)) return false;
if (!P.getarSaatJalan) { getar(pb.getarPwm); tGetar0 = millis(); }
uint32_t lewat = millis() - tGetar0;
if (lewat < pb.tGetar && !tunggu(pb.tGetar - lewat)) return false;
getarMati();
if (!tunggu(pb.tTenang)) return false; // benih mengendap 1 butir per
cekungan
if (!keZ(pb.hAmbil)) return false;
hvNyala(pb.kV); // HV hanya menyala di posisi
ambil
if (P.getarSaatAmbil) getar(pb.getarPwm); // opsional: benih "meloncat" ke
ujung sekrup
if (!tunggu(pb.tTahan)) return false;
getarMati();
return keZ(P.hAman); // benih tetap tertahan selama HV
ON
}
// Tanam: ke baris b -> turun ke lubang -> HV OFF -> benih jatuh -> naik
bool tanamBenih(uint8_t b) {
ProfilBenih& pb = PB();
if (!keX(xBaris(b))) return false;
if (!keZ(pb.hTanam)) return false;
hvMati(); // SDN LOW + bleeder 100 MOhm
tiap sekrup
if (!tunggu(pb.tLepas)) return false;
return keZ(P.hAman);
}
void mulaiPenyemaian() {
ProfilBenih& pb = PB();
char t1[8], pola[N_SEKRUP + 1];
uint8_t nOn = jumlahSekrup(P.maskSekrup);
if (nOn == 0) { layarPesan(F("Belum ada sekrup"), F("tercatat (menu 3)"), 1800);
return; }
if (xBaris(P.barisAwal + P.jumlahBaris - 1) > X_MAKS || P.xAmbil > X_MAKS) {
layarPesan(F("Baris di luar"), F("jangkauan X (kalib.)"), 2000); return;
}
lcd.clear();
snprintf(buf, sizeof(buf), "%s -%s kV", NAMA_BENIH[P.benih], fmtF(t1, pb.kV, 1));
baris(0, buf);
polaSekrup(P.maskSekrup, pola);
snprintf(buf, sizeof(buf), "%s %2u", pola, nOn); baris(1, buf);
snprintf(buf, sizeof(buf), "Baris %u..%u =%u bnh", P.barisAwal, P.barisAwal +
P.jumlahBaris - 1,
(unsigned)nOn * P.jumlahBaris); baris(2, buf);
baris(3, F("D=MULAI C=batal"));
if (tungguTombol() != 'D') return;
// medan rata-rata ujung sekrup -> pelat wadah (dibumikan) dijaga < 2,5 kV/mm (tembus
udara ~3 kV/mm)
if (pb.kV > 2.5 * pb.hAmbil) {
char t2[8];
lcd.clear();
baris(0, F("PERINGATAN: MEDAN"));
snprintf(buf, sizeof(buf), "%s kV / %s mm", fmtF(t1, pb.kV, 1), fmtF(t2, pb.hAmbil,
1)); baris(1, buf);
baris(2, F(">2,5kV/mm: bunga api"));
baris(3, F("D=tetap C=batal"));
if (tungguTombol() != 'D') return;
}

if (!homing()) { if (estop) tanganiDarurat(); else layarPesan(F("Homing gagal"),
F("Cek limit switch"), 2000); return; }
uint32_t t0 = millis();
bool ok = true;
for (uint8_t i = 0; i < P.jumlahBaris && ok; i++) {
uint8_t b = P.barisAwal + i;
lcd.clear();
snprintf(buf, sizeof(buf), "Menyemai %s", NAMA_BENIH[P.benih]); baris(0, buf);
snprintf(buf, sizeof(buf), "Baris %u/%u skr %u", i + 1, P.jumlahBaris, nOn);
baris(1, buf);
snprintf(buf, sizeof(buf), "Waktu %lus", (millis() - t0) / 1000UL); baris(2, buf);
snprintf(buf, sizeof(buf), "HV -%s kV C=stop", fmtF(t1, pb.kV, 1)); baris(3, buf);
ok = ambilBenih() && tanamBenih(b);
}
semuaAman(); // HV putus, getar mati
if (estop) { tanganiDarurat(); return; }
keZ(P.hAman); keX(0); // motor tetap ON agar sumbu Z tidak turun
lcd.clear();
if (ok) { P.totalNampan++; simpanParameter(); baris(0, F("PENYEMAIAN SELESAI")); }
else baris(0, F("DIHENTIKAN OPERATOR"));
snprintf(buf, sizeof(buf), "Waktu: %lu s", (millis() - t0) / 1000UL); baris(1, buf);
snprintf(buf, sizeof(buf), "Total nampan: %lu", P.totalNampan); baris(2, buf);
baris(3, F("Tekan tombol..."));
tungguTombol();
}
// ----------------------- MENU: PARAMETER AKTIF ----------------------
void parameterAktif() {
char a[8], b[8], c[8], pola[N_SEKRUP + 1];
uint8_t hal = 0;
while (true) {
ProfilBenih& pb = PB();
lcd.clear();
if (hal == 0) {
snprintf(buf, sizeof(buf), "Benih %s", NAMA_BENIH[P.benih]); baris(0, buf);
polaSekrup(P.maskSekrup, pola);
snprintf(buf, sizeof(buf), "%s %2u", pola, jumlahSekrup(P.maskSekrup)); baris(1,
buf);
snprintf(buf, sizeof(buf), "hAmbil %s hTnm %s", fmtF(a, pb.hAmbil, 1), fmtF(b,
pb.hTanam, 1)); baris(2, buf);
snprintf(buf, sizeof(buf), "Baris %u+%u p=%s", P.barisAwal, P.jumlahBaris, fmtF(c,
P.jarakBaris, 1)); baris(3, buf);
} else if (hal == 1) {
snprintf(buf, sizeof(buf), "vX %s vZ %s mm/s", fmtF(a, P.vX, 0), fmtF(b, P.vZ, 0));
baris(0, buf);
snprintf(buf, sizeof(buf), "Tegangan -%s kV", fmtF(a, pb.kV, 1)); baris(1, buf);
snprintf(buf, sizeof(buf), "Tahan %u Lepas %u", pb.tTahan, pb.tLepas); baris(2,
buf);
baris(3, F("A/B=hal C=kembali"));
} else {
snprintf(buf, sizeof(buf), "Getar %u%% %u ms", pb.getarPwm, pb.tGetar); baris(0,
buf);
snprintf(buf, sizeof(buf), "Tenang %u ms", pb.tTenang); baris(1, buf);
snprintf(buf, sizeof(buf), "Jalan:%s Ambil:%s", P.getarSaatJalan ? "ON" : "OFF",
P.getarSaatAmbil ? "ON" : "OFF"); baris(2, buf);
baris(3, F("A/B=hal C=kembali"));
}
char k = tungguTombol();
if (k == 'C' || k == 'D') return;
if (k == 'A') hal = (hal + 2) % 3; else hal = (hal + 1) % 3;
}
}
// --------------------- MENU: SEKRUP TERPASANG (CATATAN) -------------
// Semua soket tersambung ke HV yang sama. Menu ini hanya mencatat soket mana
// yang diisi sekrup, untuk hitungan benih/nampan dan pengingat pola pasang.
void fmtSekrup(uint8_t i, char* o) {
if (i < N_SEKRUP) { sprintf(o, "Soket %2u [%s]", i + 1, sekrupAktif(i) ? "ADA" : " -
"); return; }
switch (i - N_SEKRUP) {
case 0: strcpy(o, "Semua terpasang"); break;
case 1: strcpy(o, "Kosongkan semua"); break;
case 2: strcpy(o, "Balik pilihan"); break;
case 3: strcpy(o, "Ganjil 1,3,5..13"); break;

}
}
void menuPilihSekrup() {
lcd.clear();
baris(0, F("SEKRUP TERPASANG"));
baris(1, F("Catat soket yang"));
baris(2, F("diisi sekrup."));
baris(3, F("Tombol = lanjut"));
tungguTombol();
int8_t s = 0; char judul[21], pola[N_SEKRUP + 1];
while (true) {
polaSekrup(P.maskSekrup, pola);
snprintf(judul, sizeof(judul), "%s %2u/13", pola, jumlahSekrup(P.maskSekrup));
s = pilihMenu(judul, N_SEKRUP + 4, fmtSekrup, s);
if (s < 0) break;
if (s < N_SEKRUP) P.maskSekrup ^= (1u << s); // D atau tombol 1..9 = balik
ADA/kosong
else switch (s - N_SEKRUP) {
case 0: P.maskSekrup = MASK_SEMUA; break;
case 1: P.maskSekrup = 0; break;
case 2: P.maskSekrup ^= MASK_SEMUA; break;
case 3: P.maskSekrup = 0x1555; break; // soket 1,3,5,7,9,11,13
}
}
simpanParameter();
}
// --------------------- MENU: PARAMETER PENYEMAI ---------------------
void fmtPenyemai(uint8_t i, char* o) {
ProfilBenih& pb = PB(); char t[8];
switch (i) {
case 0: sprintf(o, "Jenis: %s", NAMA_BENIH[P.benih]); break;
case 1: sprintf(o, "Tinggi ambil %s", fmtF(t, pb.hAmbil, 1)); break;
case 2: sprintf(o, "Tinggi tanam %s", fmtF(t, pb.hTanam, 1)); break;
case 3: sprintf(o, "Jarak baris %s", fmtF(t, P.jarakBaris, 1)); break;
case 4: sprintf(o, "Kecepatan X %s", fmtF(t, P.vX, 0)); break;
case 5: sprintf(o, "Kecepatan Z %s", fmtF(t, P.vZ, 0)); break;
case 6: sprintf(o, "Baris awal %u", P.barisAwal); break;
case 7: sprintf(o, "Jumlah baris %u", P.jumlahBaris); break;
}
}
void parameterPenyemai() {
int8_t s = 0;
while ((s = pilihMenu("PARAMETER PENYEMAI", 8, fmtPenyemai, s)) >= 0) {
ProfilBenih& pb = PB();
switch (s) {
case 0: P.benih = (P.benih + 1) % N_BENIH; break;
case 1: isiAngka("Tinggi ambil (mm)", pb.hAmbil, 0.5, 30, 1); break;
case 2: isiAngka("Tinggi tanam (mm)", pb.hTanam, -8, 30, 1); break;
case 3: isiAngka("Jarak baris tray mm", P.jarakBaris, 20, 40, 2); break;
case 4: isiAngka("Kecepatan X (mm/s)", P.vX, 5, 150, 0); break;
case 5: isiAngka("Kecepatan Z (mm/s)", P.vZ, 2, 60, 0); break;
case 6: isiByte("Baris awal (1-17)", P.barisAwal, 1, 17); break;
case 7: isiByte("Jumlah baris semai", P.jumlahBaris, 1, 17); break;
}
if (P.barisAwal + P.jumlahBaris - 1 > 17) P.jumlahBaris = 18 - P.barisAwal;
terapkanKecepatan();
}
simpanParameter();
}
// --------------------- MENU: KALIBRASI STEPPER ----------------------
void jog(bool sumbuX) {
float langkah[3] = {0.1, 1.0, 10.0}; uint8_t li = 1; char t[10];
if (!sudahHome && !homing()) { if (estop) tanganiDarurat(); return; }
while (true) {
lcd.clear();
baris(0, sumbuX ? F("JOG X A=+ B=-") : F("JOG Z A=naik B=turun"));
snprintf(buf, sizeof(buf), "%s = %s mm", sumbuX ? "X" : "h", fmtF(t, sumbuX ?
xSekarang() : zSekarang(), 2)); baris(1, buf);
snprintf(buf, sizeof(buf), "Langkah %s mm", fmtF(t, langkah[li], 1)); baris(2, buf);
baris(3, F("1/2/3=0.1/1/10 C=ok"));
char k = tungguTombol();
if (k == 'C' || k == 'D') break;

if (k >= '1' && k <= '3') li = k - '1';
if (k == 'A' || k == 'B') {
float d = (k == 'A' ? 1 : -1) * langkah[li];
if (sumbuX) keX(constrain(xSekarang() + d, 0, X_MAKS));
else keZ(zSekarang() + d);
if (estop) { tanganiDarurat(); return; }
}
}
}
void fmtStepper(uint8_t i, char* o) {
char t[8];
switch (i) {
case 0: strcpy(o, "Homing"); break;
case 1: strcpy(o, "Jog sumbu X"); break;
case 2: strcpy(o, "Jog sumbu Z"); break;
case 3: strcpy(o, "Set X ambil=posisi"); break;
case 4: strcpy(o, "Set X baris1=posisi"); break;
case 5: strcpy(o, "Set h=0 (permukaan)"); break;
case 6: sprintf(o, "Langkah/mm X %s", fmtF(t, P.langkahX, 2)); break;
case 7: sprintf(o, "Langkah/mm Z %s", fmtF(t, P.langkahZ, 2)); break;
case 8: strcpy(o, "Uji ke baris n"); break;
}
}
void kalibrasiStepper() {
int8_t s = 0; float f;
while ((s = pilihMenu("KALIBRASI STEPPER", 9, fmtStepper, s)) >= 0) {
switch (s) {
case 0: if (!homing() && estop) tanganiDarurat(); break;
case 1: jog(true); break;
case 2: jog(false); break;
case 3: if (sudahHome) { P.xAmbil = xSekarang(); layarPesan(F("Posisi ambil"),
F("tersimpan"), 800); } break;
case 4: if (sudahHome) { P.xBaris1 = xSekarang(); layarPesan(F("Posisi baris 1"),
F("tersimpan"), 800); } break;
case 5: if (sudahHome) { // ujung sekrup tepat menyentuh permukaan -> h
= 0
float hLama = zSekarang();
P.hHome -= hLama;
stZ.setCurrentPosition(zLangkah(0));
layarPesan(F("Permukaan h=0"), F("tersimpan"), 800);
} break;
case 6: isiAngka("Langkah/mm X", P.langkahX, 1, 400, 2); sudahHome = false; break;
case 7: isiAngka("Langkah/mm Z", P.langkahZ, 1, 400, 2); sudahHome = false; break;
case 8: f = 1;
if (isiAngka("Uji ke baris (1-17)", f, 1, 17, 0)) {
if (!sudahHome && !homing()) break;
keZ(P.hAman); keX(xBaris((uint8_t)f)); keZ(PB().hTanam + 2);
if (estop) tanganiDarurat();
} break;
}
terapkanKecepatan();
}
simpanParameter();
}
// ------------------- MENU: KALIBRASI ELEKTROSTATIS ------------------
void fmtElektro(uint8_t i, char* o) {
char t[8]; ProfilBenih& pb = PB();
switch (i) {
case 0: sprintf(o, "Tegangan -%s kV", fmtF(t, pb.kV, 1)); break;
case 1: sprintf(o, "Waktu tahan %u ms", pb.tTahan); break;
case 2: sprintf(o, "Waktu lepas %u ms", pb.tLepas); break;
case 3: strcpy(o, "Uji tegangan tinggi"); break;
case 4: strcpy(o, "Uji ambil di wadah"); break;
}
}
bool peringatanHV() {
lcd.clear();
baris(0, F("!!TEGANGAN TINGGI!!"));
baris(1, F("Jauhkan tangan dari"));
baris(2, F("sekrup & soket HV"));
baris(3, F("#=nyalakan C=batal"));
return tungguTombol() == '#';
}

void ujiHV(float& kV) {
if (!peringatanHV()) return;
hvNyala(kV);
uint32_t t0 = millis(); char t[8];
while (millis() - t0 < 10000UL && !estop) { // mati otomatis 10 s
snprintf(buf, sizeof(buf), "HV -%s kV %lus", fmtF(t, kV, 1), 10 - (millis() - t0) /
1000UL); baris(0, buf);
snprintf(buf, sizeof(buf), "Sekrup tercatat: %u", jumlahSekrup(P.maskSekrup));
baris(1, buf);
baris(2, F("A=+0.5kV B=-0.5kV"));
baris(3, F("C=matikan"));
char k = keypad.getKey();
if (k == 'C') break;
if (k == 'A' || k == 'B') { kV = constrain(kV + (k == 'A' ? 0.5 : -0.5), 0.0,
KV_MAKS); hvNyala(kV); }
}
semuaAman();
if (estop) tanganiDarurat();
}
// Uji satu siklus ambil di atas wadah: operator menghitung benih di tiap sekrup
void ujiAmbilWadah() {
if (!peringatanHV()) return;
if (!sudahHome && !homing()) { if (estop) tanganiDarurat(); return; }
if (ambilBenih()) {
lcd.clear();
baris(0, F("Hitung benih pada"));
baris(1, F("tiap sekrup."));
baris(2, F("HV ON maks 20 s"));
baris(3, F("Tombol = lepaskan"));
uint32_t t0 = millis();
while (millis() - t0 < 20000UL && !estop && !keypad.getKey()) {}
keZ(PB().hAmbil + 3.0); // lepas kembali di atas wadah
}
hvMati();
tunggu(PB().tLepas);
keZ(P.hAman);
semuaAman();
if (estop) tanganiDarurat();
}
void kalibrasiElektro() {
int8_t s = 0;
while ((s = pilihMenu("KALIBRASI ELEKTROST.", 5, fmtElektro, s)) >= 0) {
ProfilBenih& pb = PB();
switch (s) {
case 0: isiAngka("Tegangan kV (0-10)", pb.kV, 0, KV_MAKS, 1); pb.kV = round(pb.kV
* 2) / 2.0; break; // langkah 0,5 kV
case 1: isiInt("Waktu tahan (ms)", pb.tTahan, 50, 5000); break;
case 2: isiInt("Waktu lepas (ms)", pb.tLepas, 50, 5000); break;
case 3: ujiHV(pb.kV); break;
case 4: ujiAmbilWadah(); break;
}
}
simpanParameter();
}
// ----------------------- MENU: PENGGETAR WADAH ----------------------
void fmtGetar(uint8_t i, char* o) {
ProfilBenih& pb = PB();
switch (i) {
case 0: sprintf(o, "Intensitas %u %%", pb.getarPwm); break;
case 1: sprintf(o, "Lama getar %u ms", pb.tGetar); break;
case 2: sprintf(o, "Jeda tenang %u ms", pb.tTenang); break;
case 3: sprintf(o, "Saat jalan: %s", P.getarSaatJalan ? "ON" : "OFF"); break;
case 4: sprintf(o, "Saat ambil: %s", P.getarSaatAmbil ? "ON" : "OFF"); break;
case 5: strcpy(o, "Uji penggetar"); break;
}
}
void ujiGetar() {
ProfilBenih& pb = PB();
uint8_t pwm = pb.getarPwm;
getar(pwm);
while (!estop) {
snprintf(buf, sizeof(buf), "GETAR ON %u %%", pwm); baris(0, buf);
baris(1, F("A=+10% B=-10%"));

baris(2, F("D=simpan nilai"));
baris(3, F("C=matikan"));
char k = tungguTombol();
if (k == 'C') break;
if (k == 'D') { pb.getarPwm = pwm; break; }
if (k == 'A' && pwm <= 90) pwm += 10;
if (k == 'B' && pwm >= 20) pwm -= 10;
getar(pwm);
}
getarMati();
if (estop) tanganiDarurat();
}
void menuPenggetar() {
int8_t s = 0;
while ((s = pilihMenu("PENGGETAR WADAH", 6, fmtGetar, s)) >= 0) {
ProfilBenih& pb = PB();
switch (s) {
case 0: isiByte("Intensitas getar %", pb.getarPwm, 0, 100); break;
case 1: isiInt("Lama getar (ms)", pb.tGetar, 0, 10000); break;
case 2: isiInt("Jeda tenang (ms)", pb.tTenang, 0, 5000); break;
case 3: P.getarSaatJalan ^= 1; break;
case 4: P.getarSaatAmbil ^= 1; break;
case 5: ujiGetar(); break;
}
}
simpanParameter();
}
// ---------------------- MENU: PENGATURAN SISTEM ---------------------
void fmtSistem(uint8_t i, char* o) {
char t[8];
switch (i) {
case 0: strcpy(o, "Simpan parameter"); break;
case 1: strcpy(o, "Kembali ke default"); break;
case 2: sprintf(o, "Percepatan X %s", fmtF(t, P.aX, 0)); break;
case 3: sprintf(o, "Percepatan Z %s", fmtF(t, P.aZ, 0)); break;
case 4: sprintf(o, "Tinggi aman %s", fmtF(t, P.hAman, 1)); break;
case 5: strcpy(o, "Uji keluaran"); break;
case 6: strcpy(o, "Info sistem"); break;
}
}
void ujiKeluaran() { // tanpa tegangan tinggi (SDN tetap LOW)
lcd.clear(); baris(0, F("UJI KELUARAN"));
baris(1, F("Relay HV (SDN OFF)"));
relay(PIN_RLY_HV, true); digitalWrite(PIN_LAMPU_HV, HIGH); delay(1000);
relay(PIN_RLY_HV, false); digitalWrite(PIN_LAMPU_HV, LOW);
baris(1, F("Penggetar 1 s"));
getar(PB().getarPwm); delay(1000); getarMati();
if (estop) tanganiDarurat();
}
void pengaturanSistem() {
int8_t s = 0;
while ((s = pilihMenu("PENGATURAN SISTEM", 7, fmtSistem, s)) >= 0) {
switch (s) {
case 0: simpanParameter(); layarPesan(F("Parameter"), F("tersimpan di EEPROM"),
800); break;
case 1: lcd.clear(); baris(1, F("Reset ke default?")); baris(3, F("D=ya
C=tidak"));
if (tungguTombol() == 'D') { uint32_t tot = P.totalNampan; setDefault();
P.totalNampan = tot; simpanParameter(); } break;
case 2: isiAngka("Percepatan X mm/s2", P.aX, 50, 3000, 0); break;
case 3: isiAngka("Percepatan Z mm/s2", P.aZ, 20, 1500, 0); break;
case 4: isiAngka("Tinggi aman (mm)", P.hAman, H_AMAN_MIN, 70, 1); break;
case 5: ujiKeluaran(); break;
case 6: lcd.clear();
baris(0, F("Penyemai ES " FW_VERSI));
baris(1, F("13 soket + 1 relay"));
snprintf(buf, sizeof(buf), "Total nampan %lu", P.totalNampan); baris(2,
buf);
baris(3, F("Tekan tombol..."));
tungguTombol(); break;
}
terapkanKecepatan();
}

simpanParameter();
}
// ------------------------------ MAIN --------------------------------
void fmtUtama(uint8_t i, char* o) {
static const char* const M[8] = {"1 Mulai Penyemaian", "2 Parameter Aktif", "3 Sekrup
Terpasang",
"4 Param. Penyemai", "5 Kalibrasi Stepper", "6 Kalib.
Elektrostat",
"7 Penggetar Wadah", "8 Pengaturan Sistem"};
strcpy(o, M[i]);
}
void setup() {
// keluaran berbahaya dibuat aman lebih dulu
pinMode(PIN_HV_CTRL, OUTPUT); analogWrite(PIN_HV_CTRL, 0);
pinMode(PIN_HV_SDN, OUTPUT); digitalWrite(PIN_HV_SDN, LOW);
pinMode(PIN_RLY_HV, OUTPUT); relay(PIN_RLY_HV, false);
pinMode(PIN_GETAR, OUTPUT); analogWrite(PIN_GETAR, 0);
pinMode(PIN_LAMPU_HV, OUTPUT); digitalWrite(PIN_LAMPU_HV, LOW);
pinMode(PIN_EN, OUTPUT); digitalWrite(PIN_EN, HIGH);
pinMode(PIN_LIM_X, INPUT_PULLUP);
pinMode(PIN_LIM_Z, INPUT_PULLUP);
pinMode(PIN_ESTOP, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), isrEstop, RISING);
if (digitalRead(PIN_ESTOP) == HIGH) estop = true;
muatParameter();
if (P.hAman < H_AMAN_MIN) P.hAman = H_AMAN_MIN;
stX.setPinsInverted(false, false, false);
stZ.setPinsInverted(false, false, false); // balik bila arah Z terbalik
terapkanKecepatan();
lcd.init(); lcd.backlight();
lcd.createChar(1, chrPenuh);
lcd.createChar(2, chrKosong);
lcd.clear();
baris(0, F("PENYEMAI BENIH"));
baris(1, F("ELEKTROSTATIK"));
baris(2, F("13 Soket + Penggetar"));
baris(3, F(FW_VERSI " IPB University"));
delay(1500);
}
void loop() {
if (estop) tanganiDarurat();
static int8_t s = 0;
int8_t pil = pilihMenu("== MENU UTAMA ==", 8, fmtUtama, s < 0 ? 0 : s);
if (pil < 0) return;
s = pil;
switch (pil) {
case 0: mulaiPenyemaian(); break;
case 1: parameterAktif(); break;
case 2: menuPilihSekrup(); break;
case 3: parameterPenyemai(); break;
case 4: kalibrasiStepper(); break;
case 5: kalibrasiElektro(); break;
case 6: menuPenggetar(); break;
case 7: pengaturanSistem(); break;
}
// motor tetap ber-ENABLE setelah homing: rack & pinion tidak mengunci sendiri (Z bisa
turun)
}

# 🎛️ DynamicEQ — 10-Band Parametric APO Equalizer for iOS

![iOS 14.0+](https://img.shields.io/badge/iOS-14.0%2B-blue.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)
![Build](https://img.shields.io/badge/Build-GitHub_Actions-orange.svg)

Системный параметрический 10-полосный эквалайзер для iOS-приложений (Яндекс Музыка, VK Музыка, Spotify, YouTube Music и др.) без необходимости джейлбрейка!

## ⚡ Особенности
- **64-битный DSP движок (Biquad IIR):** Студийная точность расчетов частот.
- **Аналоговая татурация `tanh()`:** Полная защита от хрипов, клиппинга и цифровых прострелов.
- **Инфра-бас (Low-Shelf 20-55Hz):** Физическое вибрационное ощущение сабвуфера в ушах.
- **iOS Control Center UI:** Размытый стеклянный фон (Glassmorphism), тактильная вибро-отдача (Haptics).
- **Пресеты и Код-Импорт:** Сохранение пресетов в память + быстрая передача конфигов через код буфера обмена.
- **Bypass Mode:** Мгновенная кнопка сравнения звука `[EQ: ON/OFF]`.
- **Живой VU-Meter:** Индикатор пиков и статуса обработки звука в реальном времени.

## 📦 Установка (.dylib)
1. Зайдите в раздел **Actions** -> Скачайте артефакт **`DynamicEQ.dylib`**.
2. Используйте утилиту подписи (**ESign, TrollStore, Scarlet, Sideloadly, Feather и др.**).
3. Инжектируйте `DynamicEQ.dylib` в ваш `.ipa` файл и установите на iOS.

## 🛠️ Сборка
Сборка происходит автоматически при каждом коммите через GitHub Actions (см. `.github/workflows/build.yml`).

## 📜 Лицензия
MIT License. Created for iOS Audio Enthusiasts.

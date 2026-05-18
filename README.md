# Parcial 3 — Diego Solís  
## Semáforo con paso de tren y cruce peatonal (STM32 Nucleo L053R8)

> Proyecto en C para STM32L053R8 que controla un semáforo con **dos botones físicos** (Tren y Peatón), un **keypad 4×4** para configurar los tiempos de **ROJO (MM:SS)** y una **LCD 16×2** para la interfaz. El conteo se muestra en un **display 7-segmentos (4 dígitos)** y un **stepper 28BYJ-48** acompaña el cambio de estado (verde/rojo). Todo funciona con **timers e interrupciones** (sin delays bloqueantes), y envía **logs** por USART.

---

## 1) Qué hace (en una línea)
- **Desde VERDE:** eliges **Tren** o **Peatón** (botones).  
- **AMARILLO (3 s):** aviso/precaución.  
- **ROJO (MM:SS):** tiempo configurable por menú (keypad + LCD).  
- **Regresa a VERDE** cuando termina el conteo.

---

## 2) Ficha técnica rápida

**MCU:** STM32L053R8  
**Comunicación:** USART2 @ 9600 baudios (PA2/PA3 AF4)  
**Interfaz usuario:** LCD 16×2 (4-bit), Keypad 4×4, 2 botones (Tren/Peatón)  
**Indicadores:** LEDs Rojo/Amarillo/Verde, Buzzer  
**Actuador:** Stepper 28BYJ-48 (half-step)  
**Display tiempo:** 7-segmentos 4 dígitos (MM:SS)  
**Arquitectura:** ISR-driven (TIM21 = “tick” 1 kHz, TIM22 = multiplex 7-seg ~2 kHz)

---

## 3) Mapa de pines (del código)

| Módulo / Señal                | MCU Pin(es)                                  | Nota |
|------------------------------|----------------------------------------------|------|
| **LCD 16×2 (4-bit)**         | RS=PA0, E=PA1, D4=PA8, D5=PA10, D6=PA5, D7=PA6 | Cola no bloqueante para comandos/datos |
| **Keypad 4×4**               | Filas PB8..PB11 (IN + pull-up) \| Cols PB12..PB15 (OUT) | Teclas: A/B/C/D, 0–9, `*`, `#` |
| **7-segmentos (4 díg.)**     | Segmentos PB0..PB7 \| Dígitos PC5, PC6, PC8, PC9 | Multiplex vía TIM22 |
| **Stepper 28BYJ-48**         | PC3, PC4, PC7, PC11                           | Secuencia half-step (tabla de 8) |
| **LEDs semáforo**            | Rojo=PC0, Amarillo=PC1, Verde=PC2             | Control directo por GPIO |
| **Buzzer**                   | PA9                                           | ON durante modo Tren |
| **Botón Tren**               | PA4 (EXTI)                                    | Disparo desde VERDE |
| **Botón Peatón**             | PA12 (EXTI)                                   | Disparo desde VERDE |
| **USART2**                   | PA2 (TX), PA3 (RX) — AF4                      | Logs/depuración |

> **Alimentación:** lógica 5 V; **GND común** entre módulos.

---

## 4) Flujo de uso

1. **Pantalla inicial (VERDE):** LCD muestra *“Libre — A: Tren / B: Peatón”*.  
2. **Disparo:** presiona **PA4 (Tren)** o **PA12 (Peatón)** → entra a **AMARILLO**.  
3. **AMARILLO (3 s):** LED Amarillo ON; en Tren suena el **buzzer**. Se prepara la **posición del stepper** para ROJO.  
4. **ROJO (MM:SS):**  
   - **Tren:** LCD *“Tren pasando”*, buzzer ON, conteo en 7-seg.  
   - **Peatón:** LCD *“Paso peatonal”*, buzzer OFF, conteo en 7-seg.  
   - Al terminar el tiempo → **Vuelve a VERDE**.  
5. **Configurar tiempos (desde VERDE):**  
   - Tecla **A** → menú **Tiempo Tren (MM:SS)**.  
   - Tecla **B** → menú **Tiempo Peatón (MM:SS)**.  
   - Ingresa **MM:SS** con el keypad; `*` borra; `#` **guarda** (valida `SS < 60`).

---

## 5) Arquitectura de temporizadores e IRQ

| Componente        | Frecuencia aprox. | Rol principal |
|-------------------|-------------------|---------------|
| **TIM21 (IRQ)**   | **1 kHz**         | *Heart-beat* del sistema: servicio LCD asíncrona, escaneo de columnas del keypad, antirrebote por FSM, servicio del **stepper**, contadores de **AMARILLO/ROJO** y actualización MM:SS (por divisor de 1 s) |
| **TIM22 (IRQ)**   | **~2 kHz**        | Multiplex del **7-segmentos** (solo en ROJO) |
| **EXTI4_15 (IRQ)**| Por flanco        | Botones **Tren/Peatón** y detección de filas del keypad |
| **USART2 (IRQ)**  | 9600 baudios      | Envío por **cola TX** (no bloquea); RX se limpia |

---

## 6) Interfaz en LCD (mensajes)
- **VERDE:** “Libre — A: Tren / B: Peatón”  
- **AMARILLO:** “Precaución — Esperando a Rojo”  
- **ROJO (TREN):** “Tren pasando”  
- **ROJO (PEATÓN):** “Paso peatonal”  
- **Menús:** “Tiempo Tren (MM:SS)” / “Tiempo Peatón (MM:SS)”

---

## 7) Parámetros por defecto
- **ROJO — Tren:** `30 s`  
- **ROJO — Peatón:** `20 s`  
- **AMARILLO:** `3 s` fijo  
- **Baudios:** `9600` (USART2)

---

## 8) Detalles de implementación que marcan la diferencia
- **LCD no bloqueante:** cola de comandos/datos + mini-máquina de estados que envía nibbles en el tick de 1 ms.  
- **Keypad robusto:** escaneo por columnas, EXTI en filas y **antirrebote** por FSM (`K_IDLE`, `K_DEB_PRESS`, `K_HELD`).  
- **Stepper suave:** *half-step* con objetivo de pasos (`stp_target_steps`) y avance periódico (cada ~3 ms).  
- **Display claro:** conversión a **MM:SS** y tabla `seg_lut` para segmentos.

---

## 9) Registro por puerto serie (logs)
- Anuncia **estado**: `VERDE`, `AMARILLO`, `ROJO`.  
- En menú, reporta cambios: `MENU: Tren / Peaton` y tiempos guardados tipo `TREN tiempo: MM:SS`.

---

## 10) Videos 


Explicacion del codigo: https://youtu.be/DMOxnEH23hs
Video de maqueta funcionando: https://youtube.com/shorts/2vojuXmkTIQ

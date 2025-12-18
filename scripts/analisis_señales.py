import serial
import time
import os
import numpy as np
import csv
import matplotlib
matplotlib.use('Agg') # MODO SILENCIOSO (Rápido, no abre ventanas)
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, medfilt

# %% CONFIGURACIÓN
puerto = "COM3"  # <--- CONFIRMA TU PUERTO
baud = 115200

condiciones = ["reposo", "vibracion", "motor"]
numEnsayos = 15      
numMuestras = 200    
fs = 100.0       
nyq = 0.5 * fs
fc = 5.0         

# --- DEFINICIÓN DE FILTROS ---
# 1. Butterworth
b_butt, a_butt = butter(4, fc / nyq, btype='low')

# 2. EMA (Exponencial)
alpha_ema = 0.15 # Ajustable

def aplicar_ema(data, alpha):
    ema_data = np.zeros_like(data)
    ema_data[0] = data[0]
    for i in range(1, len(data)):
        ema_data[i] = alpha * data[i] + (1 - alpha) * ema_data[i-1]
    return ema_data

# 3. Mediana (ya importado de scipy)

estadisticas_globales = []

print(f"Iniciando Experimento (Código Corregido)...")

try:
    # Inicializamos la conexión Serial como 's'
    s = serial.Serial(puerto, baud, timeout=1.0)
    time.sleep(2) 
    s.reset_input_buffer()

    baseFolder = "Proyecto_Final_Graficas_Paper"
    os.makedirs(baseFolder, exist_ok=True)
    
    for condicion in condiciones:
        print(f"\n========================================")
        print(f"   CONDICIÓN: '{condicion.upper()}'")
        print(f"========================================")
        
        banco_señales = [] 
        
        for i in range(3, 0, -1):
            print(f"Iniciando en {i}...", end='\r')
            time.sleep(1)
        print("\n¡CAPTURANDO 15 ENSAYOS!...")

        path_condicion = os.path.join(baseFolder, condicion)
        os.makedirs(path_condicion, exist_ok=True)

        for e in range(numEnsayos):
            print(f"  -> Leyendo Ensayo {e + 1}/{numEnsayos}...", end='\r')
            
            datos = np.zeros((numMuestras, 7))
            m = 0
            start_time = time.time()
            
            # Usamos 's' para leer del puerto (IMPORTANTE: NO SOBRESCRIBIR 's')
            s.reset_input_buffer()

            while m < numMuestras:
                try:
                    line = s.readline().decode('utf-8', errors='ignore').strip()
                    if not line: continue
                    parts = line.split(",")
                    if len(parts) != 6: continue
                    datos[m, 0] = time.time() - start_time
                    datos[m, 1:] = [float(x) for x in parts]
                    m += 1
                except ValueError: continue
                if (time.time() - start_time) > 8: break

            if m > 10:
                raw = datos[:m, 3] / 16384.0
                
                # Guardar señal recortada para graficar luego
                if len(raw) >= 180:
                    banco_señales.append(raw[:180])

                # Filtros y Estadísticas
                y_butt = filtfilt(b_butt, a_butt, raw)
                y_ema = aplicar_ema(raw, alpha_ema)
                y_med = medfilt(raw, kernel_size=5)
                
                rms_val = np.sqrt(np.mean(raw**2))
                var_val = np.var(raw)
                
                estadisticas_globales.append({
                    "Condicion": condicion, "Ensayo": e+1,
                    "RMS": round(rms_val, 4), "Varianza": round(var_val, 6)
                })

        # --- GENERAR GRÁFICAS AL TERMINAR LA CONDICIÓN ---
        if banco_señales:
            print(f"\n   Generando Gráficas para {condicion}...")
            
            matriz = np.array(banco_señales)
            senal_promedio = np.mean(matriz, axis=0)
            
            filt_butt = filtfilt(b_butt, a_butt, senal_promedio)
            filt_ema = aplicar_ema(senal_promedio, alpha_ema)
            filt_med = medfilt(senal_promedio, kernel_size=5)

            # GRAFICA 1: COMPARACION FILTROS
            fig1, ax = plt.subplots(figsize=(12, 6))
            t = np.arange(len(senal_promedio))
            
            ax.plot(t, senal_promedio, color='lightgray', linewidth=3, label='Señal Cruda')
            ax.plot(t, filt_butt, color='blue', linewidth=1.5, label='Butterworth')
            ax.plot(t, filt_ema, color='green', linestyle='--', linewidth=2, label='EMA')
            ax.plot(t, filt_med, color='red', linewidth=1.5, alpha=0.8, label='Mediana')
            
            ax.set_title(f"Comparación de Filtros - {condicion.upper()}")
            ax.set_ylabel("Aceleración (g)")
            ax.legend()
            ax.grid(True, linestyle=':')
            
            plt.savefig(os.path.join(baseFolder, f"COMPARACION_FILTROS_{condicion}.png"), dpi=150)
            plt.close(fig1)

            # GRAFICA 2: REPETITIVIDAD
            fig2, ax = plt.subplots(figsize=(10, 5))
            
            # --- CORRECCIÓN AQUÍ: Usamos 'traza' en vez de 's' ---
            for traza in banco_señales: 
                ax.plot(traza, color='gray', alpha=0.2, linewidth=1)
                
            ax.plot(senal_promedio, color='black', linewidth=1.5, linestyle='--', label='Promedio')
            ax.set_title(f"Repetitividad (15 Ensayos) - {condicion.upper()}")
            ax.set_ylabel("Aceleración (g)")
            ax.legend()
            ax.grid(True)
            
            plt.savefig(os.path.join(baseFolder, f"REPETITIVIDAD_{condicion}.png"), dpi=100)
            plt.close(fig2)

    # Guardar Excel
    if estadisticas_globales:
        keys = estadisticas_globales[0].keys()
        with open(os.path.join(baseFolder, "Tabla_Resultados_Final.csv"), 'w', newline='') as f:
            w = csv.DictWriter(f, keys)
            w.writeheader()
            w.writerows(estadisticas_globales)

    print(f"\n--- ¡TERMINADO CON ÉXITO! ---")

except Exception as e:
    print(f"ERROR: {e}")
finally:
    # Verificamos que 's' siga siendo el puerto serial antes de cerrar
    if 's' in locals() and hasattr(s, 'close') and s.is_open:
        s.close()

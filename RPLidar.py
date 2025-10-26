"""
Script mínimo para prender/apagar un RPLidar A2M12.
Uso:
  python RPLidar.py on             # enciende (arranca el motor)
  python RPLidar.py off            # apaga (detiene el motor)
  python RPLidar.py status         # muestra info / salud
  python RPLidar.py                # solicita acción en terminal
Requiere: pip install rplidar matplotlib
"""
import sys
import argparse
import time
from rplidar import RPLidar, RPLidarException
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

PORT = 'COM3'
BAUD = 256000

def connect(port, baud):
    return RPLidar(port, baudrate=baud)

def do_on(port, baud):
    lidar = None
    try:
        lidar = connect(port, baud)
        lidar.start_motor()
        print("Motor arrancado.")
    except RPLidarException as e:
        print("Error al conectar/arrancar:", e)
    finally:
        if lidar:
            try:
                lidar.disconnect()
            except:
                pass

def do_off(port, baud):
    lidar = None
    try:
        lidar = connect(port, baud)
        try:
            lidar.stop()
        except:
            pass
        lidar.stop_motor()
        print("Motor detenido.")
    except RPLidarException as e:
        print("Error al conectar/detener:", e)
    finally:
        if lidar:
            try:
                lidar.disconnect()
            except:
                pass

def do_status(port, baud):
    lidar = None
    try:
        lidar = connect(port, baud)
        info = lidar.get_info()
        health = lidar.get_health()
        print("Info:", info)
        print("Health:", health)
    except RPLidarException as e:
        print("Error al obtener estado:", e)
    finally:
        if lidar:
            try:
                lidar.disconnect()
            except:
                pass

def init_plot(panel_size=0.5, panel_pixels=128, span_m=4.0):
    """Crear ventana gráfica con ejes x,y (interactiva).
    - panel_size: tamaño de cada panel en metros (0.5 m = 50 cm)
    - panel_pixels: resolución de cada panel en píxeles (128)
    - span_m: rango total por eje en metros (por defecto 4 m -> ±2 m)
    """
    plt.ion()
    fig, ax = plt.subplots(figsize=(6,6))

    # límites centrados en 0 (ajuste opcional mediante span_m)
    half = span_m / 2.0
    ax.set_xlim(-half, half)
    ax.set_ylim(-half, half)

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('RPLidar - ejes X,Y')
    ax.set_aspect('equal', 'box')

    # Major grid cada panel_size (0.5 m), minor grid cada pixel (panel_size/panel_pixels)
    major = ticker.MultipleLocator(panel_size)
    minor = ticker.MultipleLocator(panel_size / panel_pixels)

    ax.xaxis.set_major_locator(major)
    ax.yaxis.set_major_locator(major)
    ax.xaxis.set_minor_locator(minor)
    ax.yaxis.set_minor_locator(minor)

    # Dibujar cuadricula: paneles bien visibles y subdivisión por píxeles
    ax.grid(which='major', color='gray', linewidth=1.0)
    ax.grid(which='minor', color='lightgray', linewidth=0.4, linestyle=':')

    plt.show(block=False)
    return fig, ax

def prompt_action():
    try:
        while True:
            a = input("Acción (on/off/status/exit): ").strip().lower()
            if a in ('on','off','status','exit','quit'):
                return a
            print("Acción no válida. Escriba 'on', 'off', 'status' o 'exit'.")
    except (KeyboardInterrupt, EOFError):
        print()
        return 'exit'

def main():
    p = argparse.ArgumentParser(description="Control simple RPLidar A2M12 (COM3, 256000).")
    p.add_argument("action", nargs='?', choices=['on','off','status'], help="acción a ejecutar (opcional).")
    p.add_argument("--port", default=PORT, help="Puerto serie (por defecto COM3).")
    p.add_argument("--baud", type=int, default=BAUD, help="Baudios (por defecto 256000).")
    args = p.parse_args()

    # Crear gráfico con ejes X,Y al iniciar el script
    fig, ax = init_plot()

    action = args.action
    if not action:
        action = prompt_action()
        if action in ('exit', 'quit'):
            print("Saliendo.")
            return

    if action == 'on':
        do_on(args.port, args.baud)
    elif action == 'off':
        do_off(args.port, args.baud)
    elif action == 'status':
        do_status(args.port, args.baud)

if __name__ == '__main__':
    main()
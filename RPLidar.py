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

# Tamaño del panel en metros (0.5 m = 50 cm) y resolución por panel (píxeles)
PANEL_SIZE_M = 0.5
PANEL_PIXELS = 128

# Cantidad de paneles en cada eje (configurable)
PANELS_X = 2
PANELS_Y = 2

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

def init_plot(panel_size=PANEL_SIZE_M, panel_pixels=PANEL_PIXELS, panels_x=PANELS_X, panels_y=PANELS_Y):
    """Crear ventana gráfica con ejes x,y (interactiva)."""
    # span total por eje calculado a partir del número de paneles
    span_x = panels_x * panel_size
    span_y = panels_y * panel_size

    # ajustar tamaño de figura según la relación de spans para mantener aspecto igual
    base = 6.0
    if span_y > 0:
        figsize = (max(1.0, base * (span_x / span_y)), base)
    else:
        figsize = (base, base)

    plt.ion()
    fig, ax = plt.subplots(figsize=figsize)

    ax.set_xlim(-span_x / 2.0, span_x / 2.0)
    ax.set_ylim(-span_y / 2.0, span_y / 2.0)

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('RPLidar - ejes X,Y')
    ax.set_aspect('equal', 'box')

    # Major grid cada panel_size (panel), minor grid cada pixel (panel_size/panel_pixels)
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
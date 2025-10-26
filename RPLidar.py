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

def init_plot():
    """Crear ventana gráfica con ejes x,y (interactiva)."""
    plt.ion()
    fig, ax = plt.subplots()
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('RPLidar - ejes X,Y')
    ax.set_aspect('equal', 'box')
    ax.grid(True)
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
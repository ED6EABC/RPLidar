"""
Script mínimo para prender/apagar un RPLidar A2M12.
Uso:
  python RPLidar.py on             # enciende (arranca el motor)
  python RPLidar.py off            # apaga (detiene el motor)
  python RPLidar.py status         # muestra info / salud
  python RPLidar.py scan           # muestra datos en vivo dentro del área de paneles
  python RPLidar.py                # solicita acción en terminal
Requiere: pip install rplidar matplotlib numpy
"""
import sys
import argparse
import time
import math
import threading
from collections import deque

import numpy as np
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

# Buffer máximo de puntos para que la interfaz sea fluida
POINT_BUFFER = 8000

def connect(port, baud, timeout=1.0):
    """Crear objeto RPLidar con timeout y dejar tiempo para que el dispositivo se estabilice."""
    lidar = RPLidar(port, baudrate=baud, timeout=timeout)
    # permitir que el puerto se estabilice
    time.sleep(0.05)
    # intentar limpiar buffer si existe pyserial
    ser = getattr(lidar, '_serial', None)
    if ser is not None:
        try:
            ser.reset_input_buffer()
        except AttributeError:
            try:
                ser.flushInput()
            except:
                pass
    return lidar

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
    """Crear ventana gráfica con ejes x,y (interactiva) y marcar la posición del RPLidar en (0,0)."""
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

    # Marcar la posición del RPLidar en (0,0) con un marcador y líneas de referencia
    ax.plot(0.0, 0.0, marker='o', color='blue', markersize=8, label='RPLidar (0,0)', zorder=10)
    ax.axhline(0.0, color='black', linewidth=0.8, alpha=0.6, zorder=5)
    ax.axvline(0.0, color='black', linewidth=0.8, alpha=0.6, zorder=5)
    ax.legend(loc='upper right', fontsize='small')

    plt.show(block=False)
    return fig, ax

def live_scan_and_plot(lidar, ax, stop_event,
                       panels_x=PANELS_X, panels_y=PANELS_Y,
                       panel_size=PANEL_SIZE_M, panel_pixels=PANEL_PIXELS,
                       buffer_max=POINT_BUFFER,
                       pause_s=0.02,
                       point_ttl_s=1.0,
                       angle_min_deg=0.0,
                       angle_max_deg=90.0):
    """
    Mostrar puntos en tiempo real usando la conexión 'lidar' ya abierta.
    Maneja errores de paquete ("Wrong body size") re-sincronizando el puerto.
    Solo se añaden al gráfico los puntos cuyo ángulo esté entre angle_min_deg y angle_max_deg (grados).
    """
    span_x = panels_x * panel_size
    span_y = panels_y * panel_size
    half_x = span_x / 2.0
    half_y = span_y / 2.0

    points = deque(maxlen=buffer_max)  # almacena (t, x, y)
    scatter = ax.scatter([], [], s=2, c='red', linewidths=0)

    try:
        while not stop_event.is_set():
            try:
                for scan in lidar.iter_scans():
                    if stop_event.is_set():
                        break
                    now = time.monotonic()
                    for (_, angle_deg, dist_mm) in scan:
                        if stop_event.is_set():
                            break
                        # filtrar por ángulo solicitado
                        if not (angle_min_deg <= angle_deg <= angle_max_deg):
                            continue
                        if dist_mm == 0:
                            continue
                        r = dist_mm / 1000.0  # metros
                        theta = math.radians(angle_deg)
                        x = r * math.cos(theta)
                        y = r * math.sin(theta)
                        if -half_x <= x <= half_x and -half_y <= y <= half_y:
                            points.append((now, x, y))

                    # purgar por TTL
                    cutoff = now - point_ttl_s
                    while points and points[0][0] < cutoff:
                        points.popleft()

                    if points:
                        arr = np.asarray([(px, py) for (_, px, py) in points])
                        scatter.set_offsets(arr)
                    else:
                        scatter.set_offsets(np.empty((0, 2)))

                    try:
                        ax.figure.canvas.draw_idle()
                    except Exception:
                        pass
                    plt.pause(pause_s)

                # si iter_scans terminó normalmente, hacer una pequeña espera antes de reintentar
                time.sleep(0.01)

            except RPLidarException as e:
                msg = str(e)
                # si es el error de sincronización, limpiar buffer y reintentar
                if 'Wrong body size' in msg or 'wrong body size' in msg.lower():
                    ser = getattr(lidar, '_serial', None)
                    if ser is not None:
                        try:
                            ser.reset_input_buffer()
                        except AttributeError:
                            try:
                                ser.flushInput()
                            except:
                                pass
                    # pequeña pausa para que el dispositivo vuelva a estado estable
                    time.sleep(0.05)
                    continue
                else:
                    # otros errores los informamos y salimos del bucle
                    print("RPLidarException en live scan:", e)
                    stop_event.set()
                    break

    except KeyboardInterrupt:
        stop_event.set()
    finally:
        try:
            lidar.stop()
        except:
            pass
        try:
            lidar.stop_motor()
        except:
            pass
        try:
            lidar.disconnect()
        except:
            pass

def prompt_action():
    try:
        while True:
            a = input("Acción (on/off/status/scan/exit): ").strip().lower()
            if a in ('on','off','status','scan','exit','quit'):
                return a
            print("Acción no válida. Escriba 'on', 'off', 'status', 'scan' o 'exit'.")
    except (KeyboardInterrupt, EOFError):
        print()
        return 'exit'

def main():
    p = argparse.ArgumentParser(description="Control simple RPLidar A2M12 (COM3, 256000).")
    p.add_argument("action", nargs='?', choices=['on','off','status','scan'], help="acción a ejecutar (opcional).")
    p.add_argument("--port", default=PORT, help="Puerto serie (por defecto COM3).")
    p.add_argument("--baud", type=int, default=BAUD, help="Baudios (por defecto 256000).")
    p.add_argument("--panels-x", type=int, default=PANELS_X, help="Cantidad de paneles en X.")
    p.add_argument("--panels-y", type=int, default=PANELS_Y, help="Cantidad de paneles en Y.")
    p.add_argument("--buffer", type=int, default=POINT_BUFFER, help="Buffer máximo de puntos.")
    args = p.parse_args()

    # Crear gráfico con ejes X,Y al iniciar el script (siempre se muestra)
    fig, ax = init_plot(panel_size=PANEL_SIZE_M, panel_pixels=PANEL_PIXELS,
                        panels_x=args.panels_x, panels_y=args.panels_y)

    action = args.action
    # si se pide acciones puntuales, ejecutarlas y salir
    if action == 'on':
        do_on(args.port, args.baud)
        return
    elif action == 'off':
        do_off(args.port, args.baud)
        return
    elif action == 'status':
        do_status(args.port, args.baud)
        return

    # Por defecto: iniciar RPLidar y mostrar escaneo en vivo.
    stop_event = threading.Event()
    # cerrar la ventana activa dispara el evento de parada
    fig.canvas.mpl_connect('close_event', lambda event: stop_event.set())

    lidar = None
    try:
        lidar = connect(args.port, args.baud)
        lidar.start_motor()
        print("Motor arrancado. Cerrar la ventana o presionar Ctrl+C para detener.")
        # ejecutar escaneo en el hilo principal (permite procesar eventos GUI con plt.pause)
        live_scan_and_plot(lidar, ax, stop_event,
                           panels_x=args.panels_x, panels_y=args.panels_y,
                           panel_size=PANEL_SIZE_M, panel_pixels=PANEL_PIXELS,
                           buffer_max=args.buffer)
    except RPLidarException as e:
        print("Error inicializando RPLidar:", e)
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        # por si algo quedó abierto (live_scan ya intenta limpiar)
        if lidar:
            try:
                lidar.stop()
            except:
                pass
            try:
                lidar.stop_motor()
            except:
                pass
            try:
                lidar.disconnect()
            except:
                pass
        print("Programa finalizado.")

if __name__ == '__main__':
    main()
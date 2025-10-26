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
from matplotlib.patches import Circle
import ctypes

PORT = 'COM3'
BAUD = 256000

# Tamaño del panel en metros (0.5 m = 50 cm) y resolución por panel (píxeles)
PANEL_SIZE_M = 0.5
PANEL_PIXELS = 128

# Cantidad de paneles en cada eje (configurable)
PANELS_X = 2
PANELS_Y = 1

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

def init_plot(panel_size=PANEL_SIZE_M, panel_pixels=PANEL_PIXELS, panels_x=PANELS_X, panels_y=PANELS_Y,
              origin_offset_panels_x=0, origin_offset_panels_y=0):
    """Crear ventana gráfica mostrando solo paneles positivos en x,y (desde 0 hasta span).
    El eje Y está invertido para que el origen (0,0) quede en la esquina superior izquierda.
    origin_offset_panels_x/y: cantidad de paneles a desplazar la posición del RPLidar desde la
    esquina superior izquierda (valores enteros, pueden ser 0)."""
    # span total por eje calculado a partir del número de paneles
    span_x = panels_x * panel_size
    span_y = panels_y * panel_size

    # calcular offset del RPLidar en metros
    offset_x_m = origin_offset_panels_x * panel_size
    offset_y_m = origin_offset_panels_y * panel_size

    # ajustar tamaño de figura según la relación de spans para mantener aspecto igual
    base = 6.0
    if span_y > 0:
        figsize = (max(1.0, base * (span_x / span_y)), base)
    else:
        figsize = (base, base)

    plt.ion()
    fig, ax = plt.subplots(figsize=figsize)

    # Obtener resolución de pantalla (px) — Windows: GetSystemMetrics
    try:
        screen_w = ctypes.windll.user32.GetSystemMetrics(0)
        screen_h = ctypes.windll.user32.GetSystemMetrics(1)
    except Exception:
        # fallback si no disponible
        screen_w, screen_h = 0, 0

    # Mostrar solo el cuadrante positivo: x desde 0 hasta span_x, y invertido (span_y .. 0)
    ax.set_xlim(0.0, span_x)
    ax.set_ylim(span_y, 0.0)  # invertir Y para que 0 esté arriba

    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('RPLidar - origen en esquina superior izquierda (0,0)')
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

    # Marcar la posición del RPLidar en offset (m) desde la esquina superior izquierda
    ax.plot(offset_x_m, offset_y_m, marker='o', color='blue', markersize=8, label='RPLidar (offset)', zorder=10)
    # líneas de referencia en los bordes (x=0,y=0) y en la posición del RPLidar
    ax.axhline(0.0, color='black', linewidth=0.8, alpha=0.6, zorder=5)
    ax.axvline(0.0, color='black', linewidth=0.8, alpha=0.6, zorder=5)
    # línea indicando la posición del sensor
    ax.axvline(offset_x_m, color='blue', linewidth=0.6, alpha=0.8, linestyle='--', zorder=6)
    ax.axhline(offset_y_m, color='blue', linewidth=0.6, alpha=0.8, linestyle='--', zorder=6)
    ax.legend(loc='upper right', fontsize='small')

    # Añadir etiqueta con resolución de pantalla en la esquina superior izquierda del figure
    if screen_w and screen_h:
        # calcular cuántos paneles (en X/Y) serían necesarios para cubrir la resolución de pantalla,
        # usando panel_pixels (píxeles por panel)
        panels_needed_x = int(math.ceil(screen_w / panel_pixels))
        panels_needed_y = int(math.ceil(screen_h / panel_pixels))
        fig.text(0.01, 0.99,
                 f"Screen: {screen_w} x {screen_h} px — Panels needed: {panels_needed_x} x {panels_needed_y}",
                 ha='left', va='top', fontsize=9, color='black', zorder=30)

    plt.show(block=False)
    return fig, ax

def live_scan_and_plot(lidar, ax, stop_event,
                       panels_x=PANELS_X, panels_y=PANELS_Y,
                       panel_size=PANEL_SIZE_M, panel_pixels=PANEL_PIXELS,
                       buffer_max=POINT_BUFFER,
                       pause_s=0.02,
                       point_ttl_s=1.0,
                       angle_min_deg=0.0,
                       angle_max_deg=90.0,
                       origin_offset_panels_x=0, origin_offset_panels_y=0):
    """
    Mostrar puntos en tiempo real usando la conexión 'lidar' ya abierta.
    origin_offset_panels_x/y: cantidad de paneles a desplazar la posición del RPLidar
    (se aplica sumando el offset en metros a cada punto leído).
    """
    span_x = panels_x * panel_size
    span_y = panels_y * panel_size

    # offset en metros
    offset_x_m = origin_offset_panels_x * panel_size
    offset_y_m = origin_offset_panels_y * panel_size

    points = deque(maxlen=buffer_max)  # almacena (t, x_plot, y_plot)
    scatter = ax.scatter([], [], s=20, c='red', linewidths=0)

    # etiquetas de texto para cada centro (se reemplazan en cada frame)
    label_texts = []

    def _cluster_points(points_arr, radius):
        """Agrupa puntos por distancia (algoritmo greedy): devuelve centros y cuentas."""
        if points_arr.size == 0:
            return np.empty((0, 2)), np.empty((0,), dtype=int)
        clusters = []  # cada cluster: [sum_x, sum_y, count]
        r2 = radius * radius
        for x, y in points_arr:
            placed = False
            for c in clusters:
                cx = c[0] / c[2]
                cy = c[1] / c[2]
                if (x - cx) * (x - cx) + (y - cy) * (y - cy) <= r2:
                    c[0] += x
                    c[1] += y
                    c[2] += 1
                    placed = True
                    break
            if not placed:
                clusters.append([x, y, 1])
        centers = np.array([[c[0] / c[2], c[1] / c[2]] for c in clusters])
        counts = np.array([c[2] for c in clusters], dtype=int)
        return centers, counts

    CLUSTER_RADIUS_M = 0.20  # 20 cm (agrupamiento)
    MOVEMENT_DETECT_DIST = 0.01  # 1 cm -> umbral para considerar que cambió de posición
    RADIO_MOVIMIENTO = 0.05  # 5 cm círculo verde
    MOVEMENT_CIRCLE_TTL = 0.6  # segundos que permanece el círculo

    prev_centers = None
    movement_patches = []  # lista de (Circle patch, expire_time)

    # offset para posicionar la etiqueta ligeramente por encima del punto (en metros)
    LABEL_OFFSET_M = 0.02

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
                        # coordenadas relativas al sensor
                        x_rel = r * math.cos(theta)
                        y_rel = r * math.sin(theta)
                        # convertir a coordenadas del plot (sumando offset del sensor)
                        x = x_rel + offset_x_m
                        y = y_rel + offset_y_m
                        # conservar solo puntos dentro del cuadrante positivo [0, span]
                        if 0.0 <= x <= span_x and 0.0 <= y <= span_y:
                            points.append((now, x, y))

                    # purgar por TTL
                    cutoff = now - point_ttl_s
                    while points and points[0][0] < cutoff:
                        points.popleft()

                    # remover patches expirados
                    new_movement_patches = []
                    for p, exp in movement_patches:
                        if now >= exp:
                            try:
                                p.remove()
                            except Exception:
                                pass
                        else:
                            new_movement_patches.append((p, exp))
                    movement_patches = new_movement_patches

                    # remover etiquetas antiguas (será reemplazadas por las nuevas)
                    for t in label_texts:
                        try:
                            t.remove()
                        except Exception:
                            pass
                    label_texts = []

                    if points:
                        arr = np.asarray([(px, py) for (_, px, py) in points])
                        centers, counts = _cluster_points(arr, CLUSTER_RADIUS_M)
                        if centers.size:
                            scatter.set_offsets(centers)
                            # escalar tamaño por cantidad de puntos en el cluster (visual)
                            sizes = np.clip(8 + counts * 6, 8, 200)
                            scatter.set_sizes(sizes)

                            # crear etiquetas sobre cada centro con coordenadas en píxeles
                            total_pixels_x = panels_x * panel_pixels
                            total_pixels_y = panels_y * panel_pixels
                            for c in centers:
                                # calcular posición de la etiqueta según inversión de Y del eje
                                y0, y1 = ax.get_ylim()
                                if y0 < y1:
                                    label_y = c[1] + LABEL_OFFSET_M
                                else:
                                    label_y = c[1] - LABEL_OFFSET_M

                                # convertir coordenadas en metros a píxeles (origen top-left)
                                px = int(math.floor((c[0] / panel_size) * panel_pixels))
                                py = int(math.floor((c[1] / panel_size) * panel_pixels))
                                # asegurar rango válido
                                px = max(0, min(px, total_pixels_x - 1))
                                py = max(0, min(py, total_pixels_y - 1))

                                txt = ax.text(c[0], label_y,
                                              f"({px},{py})",
                                              fontsize=7, color='black', zorder=20,
                                              ha='center', va='bottom' if y0 < y1 else 'top')
                                label_texts.append(txt)

                            # detectar movimientos comparando con prev_centers
                            if prev_centers is None or prev_centers.size == 0:
                                # primera vez: no marcar movimientos, sólo guardar
                                prev_centers = centers.copy()
                            else:
                                # para cada centro actual encontrar el prev más cercano
                                for c in centers:
                                    # calcular distancias a prev_centers
                                    dists = np.sum((prev_centers - c) ** 2, axis=1)
                                    idx = np.argmin(dists)
                                    dist = math.sqrt(dists[idx])
                                    if dist >= MOVEMENT_DETECT_DIST:
                                        # crear círculo verde en la posición nueva
                                        circ = Circle((c[0], c[1]), RADIO_MOVIMIENTO,
                                                      edgecolor='green', facecolor='none',
                                                      linewidth=1.5, zorder=12)
                                        ax.add_patch(circ)
                                        movement_patches.append((circ, now + MOVEMENT_CIRCLE_TTL))
                                prev_centers = centers.copy()
                        else:
                            scatter.set_offsets(np.empty((0, 2)))
                            scatter.set_sizes([])
                            prev_centers = None
                    else:
                        scatter.set_offsets(np.empty((0, 2)))
                        scatter.set_sizes([])
                        prev_centers = None

                    try:
                        ax.figure.canvas.draw_idle()
                    except Exception:
                        pass
                    plt.pause(pause_s)

                time.sleep(0.01)

            except RPLidarException as e:
                msg = str(e)
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
                    time.sleep(0.05)
                    continue
                else:
                    print("RPLidarException en live scan:", e)
                    stop_event.set()
                    break

    except KeyboardInterrupt:
        stop_event.set()
    finally:
        # limpiar patches y etiquetas
        for p, _ in movement_patches:
            try:
                p.remove()
            except:
                pass
        for t in label_texts:
            try:
                t.remove()
            except:
                pass
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
    p.add_argument("--origin-offset-x", type=int, default=0, help="Offset de origen en paneles (X).")
    p.add_argument("--origin-offset-y", type=int, default=0, help="Offset de origen en paneles (Y).")
    p.add_argument("--buffer", type=int, default=POINT_BUFFER, help="Buffer máximo de puntos.")
    args = p.parse_args()

    action = args.action

    # Acciones puntuales: no pedir prompt, ejecutar y salir
    if action == 'on':
        do_on(args.port, args.baud)
        return
    elif action == 'off':
        do_off(args.port, args.baud)
        return
    elif action == 'status':
        do_status(args.port, args.baud)
        return

    # Prompt para especificar cantidad de paneles en X antes de iniciar el programa
    try:
        while True:
            try:
                s = input(f"Cantidad de paneles en X [{args.panels_x}]: ").strip()
            except (KeyboardInterrupt, EOFError):
                print()
                return
            if s == '':
                panels_x = args.panels_x
                break
            try:
                panels_x = int(s)
                if panels_x < 1:
                    print("Ingrese un entero mayor o igual a 1.")
                    continue
                break
            except ValueError:
                print("Entrada inválida. Ingrese un número entero.")
    except (KeyboardInterrupt, EOFError):
        print()
        return

    # Prompt para especificar cantidad de paneles en Y antes de iniciar el programa
    try:
        while True:
            try:
                s = input(f"Cantidad de paneles en Y [{args.panels_y}]: ").strip()
            except (KeyboardInterrupt, EOFError):
                print()
                return
            if s == '':
                panels_y = args.panels_y
                break
            try:
                panels_y = int(s)
                if panels_y < 1:
                    print("Ingrese un entero mayor o igual a 1.")
                    continue
                break
            except ValueError:
                print("Entrada inválida. Ingrese un número entero.")
    except (KeyboardInterrupt, EOFError):
        print()
        return

    # Prompt para especificar offset del origen (en paneles) antes de iniciar el programa
    try:
        while True:
            try:
                s = input(f"Offset origen en paneles X [{args.origin_offset_x}]: ").strip()
            except (KeyboardInterrupt, EOFError):
                print()
                return
            if s == '':
                origin_offset_x = args.origin_offset_x
                break
            try:
                origin_offset_x = int(s)
                # permitir 0 o más (puede ajustarse si se desea permitir negativos)
                if origin_offset_x < 0:
                    print("Ingrese un entero mayor o igual a 0.")
                    continue
                break
            except ValueError:
                print("Entrada inválida. Ingrese un número entero.")
    except (KeyboardInterrupt, EOFError):
        print()
        return

    try:
        while True:
            try:
                s = input(f"Offset origen en paneles Y [{args.origin_offset_y}]: ").strip()
            except (KeyboardInterrupt, EOFError):
                print()
                return
            if s == '':
                origin_offset_y = args.origin_offset_y
                break
            try:
                origin_offset_y = int(s)
                if origin_offset_y < 0:
                    print("Ingrese un entero mayor o igual a 0.")
                    continue
                break
            except ValueError:
                print("Entrada inválida. Ingrese un número entero.")
    except (KeyboardInterrupt, EOFError):
        print()
        return

    # Crear gráfico con ejes X,Y usando los valores provistos por prompt
    fig, ax = init_plot(panel_size=PANEL_SIZE_M, panel_pixels=PANEL_PIXELS,
                        panels_x=panels_x, panels_y=panels_y,
                        origin_offset_panels_x=origin_offset_x, origin_offset_panels_y=origin_offset_y)

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
                           panels_x=panels_x, panels_y=panels_y,
                           panel_size=PANEL_SIZE_M, panel_pixels=PANEL_PIXELS,
                           buffer_max=args.buffer,
                           origin_offset_panels_x=origin_offset_x, origin_offset_panels_y=origin_offset_y)
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
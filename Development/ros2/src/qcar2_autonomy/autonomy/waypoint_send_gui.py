#!/usr/bin/env python3
"""
Enviar la ruta al coche: con ventana (tkinter) o desde terminal (Enter).
Publica en /waypoints_send_trigger para que waypoint_visual_node envíe la ruta.
Si tkinter no está instalado en el contenedor, usa modo terminal (pulsa Enter).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class WaypointSendNode(Node):
    def __init__(self):
        super().__init__("waypoint_send_gui")
        self.pub = self.create_publisher(Empty, "/waypoints_send_trigger", 10)
        self.trigger_count = 0

    def send_trigger(self):
        self.pub.publish(Empty())
        self.trigger_count += 1
        return self.trigger_count


def run_terminal_mode(node: WaypointSendNode):
    """Modo terminal: pulsa Enter para enviar la ruta (sin tkinter)."""
    print("  Waypoints QCar2 - Enviar ruta (modo terminal)")
    print("  Marca puntos en RViz (Publish Point), luego pulsa ENTER aquí para enviar la ruta.")
    print("  Ctrl+C para salir.")
    print()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            try:
                input("  >>> Pulsa ENTER para enviar ruta al coche >>> ")
            except EOFError:
                break
            n = node.send_trigger()
            print(f"  Ruta enviada ({n}).")
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


def run_gui_mode(node: WaypointSendNode):
    """Modo ventana con tkinter."""
    import tkinter as tk
    from tkinter import font as tkfont

    root = tk.Tk()
    root.title("Waypoints QCar2 - Enviar ruta")
    root.geometry("320x180")
    root.resizable(True, True)
    bg, fg, btn_bg = "#2b2b2b", "#e0e0e0", "#0d7377"
    root.configure(bg=bg)

    title_font = tkfont.Font(family="Sans", size=12, weight="bold")
    label_font = tkfont.Font(family="Sans", size=10)

    tk.Label(root, text="Waypoints QCar2", font=title_font, bg=bg, fg=fg).pack(pady=(16, 8))
    tk.Label(root, text="1. Marca puntos en RViz (Publish Point)\n2. Pulsa el botón para enviar la ruta al coche", font=label_font, bg=bg, fg=fg, justify=tk.LEFT).pack(pady=(0, 12))

    status_var = tk.StringVar(value="")
    tk.Label(root, textvariable=status_var, font=label_font, bg=bg, fg="#7fbf7f").pack(pady=(0, 8))

    def on_send():
        n = node.send_trigger()
        status_var.set(f"Ruta enviada ({n})")
        root.after(2000, lambda: status_var.set(""))

    tk.Button(root, text="Enviar ruta al coche", command=on_send, font=("Sans", 11, "bold"), bg=btn_bg, fg="white", activebackground="#0a5c5f", activeforeground="white", relief=tk.FLAT, padx=24, pady=12, cursor="hand2").pack(pady=8)

    def spin():
        rclpy.spin_once(node, timeout_sec=0.05)
        root.after(50, spin)

    root.after(50, spin)

    def on_closing():
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSendNode()

    try:
        import tkinter as tk  # noqa: F401
        run_gui_mode(node)
    except ModuleNotFoundError:
        run_terminal_mode(node)


if __name__ == "__main__":
    main()

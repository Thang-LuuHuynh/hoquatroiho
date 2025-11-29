# love_me_tkinter.py — Mini app 'Do you love me?' for desktop
# Run: python love_me_tkinter.py
import tkinter as tk
from tkinter import messagebox
import random

W, H = 520, 360

root = tk.Tk()
root.title("Do you love me? 💘")
root.geometry(f"{W}x{H}")
root.configure(bg="#ffe6ee")
root.resizable(False, False)

card = tk.Frame(root, bg="white", bd=0, highlightthickness=0)
card.place(relx=.5, rely=.5, anchor="center", width=460, height=280)
card.configure(bg="white")

title = tk.Label(card, text="Do you love me?", font=("Segoe UI", 20, "bold"), bg="white", fg="#4a2b33")
title.pack(pady=(18, 6))

note = tk.Label(card, text="Hãy trả lời thật lòng nha 💖", font=("Segoe UI", 11), bg="white", fg="#4a2b33")
note.pack(pady=(0, 10))

btn_area = tk.Frame(card, bg="white", width=360, height=80)
btn_area.pack()
btn_area.pack_propagate(False)

def on_yes():
    messagebox.showinfo("Yêu nhau nè!", "Biết mà! Em iu anh nhiều lắm 😘")

def move_no(event=None):
    btn_area.update_idletasks()
    w = btn_area.winfo_width()
    h = btn_area.winfo_height()
    bw = no_btn.winfo_width()
    bh = no_btn.winfo_height()
    x = random.randint(10, max(10, w-bw-10))
    y = random.randint(5, max(5, h-bh-5))
    if x < 120: x = min(w-bw-5, x + 160)  # keep away from Yes
    no_btn.place(x=x, y=y)

yes_btn = tk.Button(btn_area, text="Yes", command=on_yes, bg="#ff6f91", fg="white",
                    activebackground="#ff5f83", activeforeground="white",
                    relief="flat", bd=0, padx=16, pady=8)
yes_btn.place(x=20, y=20)

no_btn = tk.Button(btn_area, text="No", bg="white", fg="#333", relief="solid",
                   bd=1, padx=16, pady=8)
no_btn.place(x=120, y=20)

no_btn.bind("<Enter>", move_no)     # hover (desktop)
no_btn.bind("<Button-1>", move_no)  # click (mobile emulation)

root.mainloop()

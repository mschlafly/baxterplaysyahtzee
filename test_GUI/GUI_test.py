

import Tkinter as tk

window = tk.Tk()
window.title('my window')
WINDOW_WIDTH=600
window.geometry('600x800')

# Vars
TEST=True

# Settings
def gap():
    GAP_COLOR="red"
    GAP_HEIGHT=1
    tk.Label(window, text="", bg=GAP_COLOR, font=('Arial', 12), width=WINDOW_WIDTH, height=GAP_HEIGHT).pack()
def make_label(text):
    item = tk.Label(window, text=text, fg="black", bg="white",
        font=('Arial', 12), width=WINDOW_WIDTH, height=2)
    item.pack()
    return item

def make_input(text0=""):
    item = tk.Text(window, height=2)
    item.insert('end',text0)
    item.pack()
    return item

def make_output(text0=""):
    item = tk.Text(window, height=2)
    item.insert('end',text0)
    item.pack()
    return item

def make_button(test, command):
    item = tk.Button(window, text=test, width=15, height=2, command=command)
    item.pack()
    return item

# Input and Output
def set_text(item, test):
    item.delete('1.0', "end-1c")
    item.insert('end',test)
    return

def get_text(item):
    item.delete('1.0', "end-1c")
    return item.get("1.0","end-1c")


# ================================================
# Show joint angles
gap()
show_joint_l0=make_label("Baxter's current joint angles are:")
show_joint_Out=make_output()

# ================================================
# Show end-effector poses
gap()
show_end_l0=make_label("Baxter's current end-effector Position and Angle are:")
show_end_OutP=make_output()
show_end_OutR=make_output()

# ================================================
# forward kinamtics
fk_OutP_TEXT="EndPose = "
fk_OutR_TEXT="EndAngle= "
def fk_func():
    if TEST:
        set_text(fk_OutP, fk_OutP_TEXT+"1,1,1")
        set_text(fk_OutR, fk_OutR_TEXT+"1,2,3")
    else:
        None
gap()
fk_bt = make_button("Compute FK",fk_func)
fk_l0 = make_label("Forward kinematics. Input 7 joint values seperated by comma \',\':")
fk_In = make_input()
fk_l1 = make_label("The end effector Position and Angle is:")
fk_OutP = make_output(fk_OutP_TEXT)
fk_OutR = make_output(fk_OutR_TEXT)

# ================================================
# inverse kinamtics
ik_InP_TEXT="EndPose = "
ik_InR_TEXT="EndAngle= "
def ik_func():
    if TEST:
        set_text(ik_Out, "ik result")
    else:
        None
gap()
ik_bt = make_button("Compute IK",ik_func)
ik_l0 = make_label("Inverse kinematics. Input Position and Angle of end-effector seperated by comma \',\':")
ik_In = make_input(ik_InP_TEXT)
ik_In = make_input(ik_InR_TEXT)
ik_l1 = make_label("The joint angles are:")
ik_Out = make_output()

window.mainloop()
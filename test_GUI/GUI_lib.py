#!/usr/bin/python

import Tkinter as tk

# Vars
WINDOW_WIDTH = 600

ik_InP_TEXT = "EndPose = "
ik_InR_TEXT = "EndAngle= "

fk_OutP_TEXT = "EndPose = "
fk_OutR_TEXT = "EndAngle= "

# ================================================
class myTkGUI(object):
# ================================================
    def __init__(self):
        self.window = tk.Tk()
        self.window.title('Baxter Param Interface')
        self.window.geometry('600x900')

        # ================================================
        # Show joint angles
        self.gap()
        self.show_joint_l0 = self.make_label("Baxter's current joint angles are:")
        self.show_joint_Out = self.make_output()

        # ================================================
        # Show end-effector poses
        self.gap()
        self.show_end_l0 = self.make_label(
            "Baxter's current end-effector Position and Angle are:")
        self.show_end_OutP = self.make_output()
        self.show_end_OutR = self.make_output()

        # ================================================
        # forward kinamtics
        self.gap()
        self.fk_bt = self.make_button("Compute FK", self.fk_func)
        self.fk_l0 = self.make_label(
            "Forward kinematics. Input 7 joint values seperated by comma \',\':")
        self.fk_In = self.make_input()
        self.fk_l1 = self.make_label("The end effector Position and Angle is:")
        self.fk_OutP = self.make_output(fk_OutP_TEXT)
        self.fk_OutR = self.make_output(fk_OutR_TEXT)

        # ================================================
        # inverse kinamtics
        self.gap()
        self.ik_bt = self.make_button("Compute IK", self.ik_func)
        self.ik_l0 = self.make_label(
            "Inverse kinematics. Input Position and Angle of end-effector seperated by comma \',\':")
        self.ik_In = self.make_input(ik_InP_TEXT)
        self.ik_In = self.make_input(ik_InR_TEXT)
        self.ik_l1 = self.make_label("The joint angles are:")
        self.ik_Out = self.make_output()


        # ================================================
        # button for closing
        self.make_button("Close",self.func_close_window)

    # ------------------------------ callback functions ------------------------------
    def fk_func(self):
        self.set_text(self.fk_OutP, fk_OutP_TEXT+"1,1,1")
        self.set_text(self.fk_OutR, fk_OutR_TEXT+"1,2,3")

    def ik_func(self):
        self.set_text(self.ik_Out, "ik result")

    def func_close_window(self):
        self.window.destroy()

    # ------------------------------ Settings ------------------------------

    def gap(self):
        GAP_COLOR = "red"
        GAP_HEIGHT = 1
        tk.Label(self.window, text="", bg=GAP_COLOR, font=('Arial', 12),
                width=WINDOW_WIDTH, height=GAP_HEIGHT).pack()


    def make_label(self, text):
        item = tk.Label(self.window, text=text, fg="black", bg="white",
                        font=('Arial', 12), width=WINDOW_WIDTH, height=2)
        item.pack()
        return item


    def make_input(self, text0=""):
        item = tk.Text(self.window, height=2)
        item.insert('end', text0)
        item.pack()
        return item


    def make_output(self, text0=""):
        item = tk.Text(self.window, height=2)
        item.insert('end', text0)
        item.pack()
        return item


    def make_button(self, test, command):
        item = tk.Button(self.window, text=test, width=15, height=2, command=command)
        item.pack()
        return item

    # Input and Output

    def set_text(self, item, test):
        item.delete('1.0', "end-1c")
        item.insert('end', test)
        return

    def get_text(self, item):
        item.delete('1.0', "end-1c")
        return item.get("1.0", "end-1c")


if __name__=="__main__":
    gui=myTkGUI()
    gui.window.mainloop()

#!/usr/bin/python

# Test my Baxter GUI


import Tkinter as tk

# Vars
WINDOW_WIDTH = 600

ik_InP_TEXT = "EndPose  = 0.582583, -0.180819, 0.216003"
# ik_InR_TEXT = "EndAngle = 0, 0, 0"
ik_InR_TEXT = "EndAngle = 3.02, 0.16, 3.06"

fk_OutP_TEXT = "EndPose  = "
fk_OutR_TEXT = "EndAngle = "

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
            "Baxter's current end-effector Position and Euler-Angle are:")
        self.show_end_OutP = self.make_output()
        self.show_end_OutR = self.make_output()

        # ================================================
        # forward kinamtics
        self.gap()
        self.fk_bt = self.make_button("Compute FK", self.button_callback_fk_func)
        self.fk_l0 = self.make_label(
            "(Not done yet!) Forward kinematics. Input 7 joint values seperated by comma \',\':")
        self.fk_In = self.make_input()
        self.fk_l1 = self.make_label("The end effector Position and Euler-Angle are:")
        self.fk_OutP = self.make_output(fk_OutP_TEXT)
        self.fk_OutR = self.make_output(fk_OutR_TEXT)

        # ================================================
        # inverse kinamtics
        self.gap()
        self.ik_bt = self.make_button("Compute IK", self.button_callback_ik_compute)
        self.ik_bt2 = self.make_button("Move robot to there", self.button_callback_ik_move_robot)

        self.ik_l0 = self.make_label(
            "Inverse kinematics. Input Position and Euler-Angle of end-effector seperated by comma \',\':")
        self.ik_InP = self.make_input(ik_InP_TEXT)
        self.ik_InR = self.make_input(ik_InR_TEXT)
        self.ik_l1 = self.make_label("The joint angles are:")
        self.ik_Out = self.make_output()


        # ================================================
        # button for closing
        self.gap()
        self.make_button("Close",self.func_close_window)

    # ------------------------------ button_callback functions ------------------------------
    def button_callback_fk_func(self):
        self.set_text(self.fk_OutP, fk_OutP_TEXT+"I haven't implemented it yet!")
        self.set_text(self.fk_OutR, fk_OutR_TEXT+"I haven't implemented it yet!")

    def button_callback_ik_compute(self):
        self.set_text(self.ik_Out, "ik result")

    def button_callback_ik_move_robot(self):
        print("Move the robot")
        # you should overload this function in another file when you import this class

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
        item = tk.Text(self.window, height=1)
        item.insert('end', text0)
        item.pack()
        return item


    def make_output(self, text0=""):
        item = tk.Text(self.window, height=1)
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
        return item.get("1.0", "end-1c")


if __name__=="__main__":
    gui=myTkGUI()
    gui.window.mainloop()

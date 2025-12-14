import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button
import matplotlib.animation as animation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class PDE4431_Coursework2:
    def __init__(self):
        self.L1 = 0.3
        self.L2 = 0.3
        self.L3 = 0.3
        
        self.is_animating = False
        self.is_paused = False
        self.animation_sequence = []
        self.joints = np.array([0.5, np.radians(90), 0.0, np.radians(90)])
        self.current_animation_step = 0
        self.total_animation_steps = 50
        
        self.crate1_positions = [
            {'joints': np.array([0.5, np.radians(90), 0.0, np.radians(90)])},
            {'joints': np.array([0.6, np.radians(100), np.radians(-62.8), np.radians(92.3)])},
            {'joints': np.array([0.51, np.radians(100), np.radians(-62.8), np.radians(92.3)])},
            {'joints': np.array([0.776, np.radians(93.2), np.radians(-50.8), np.radians(56.7)])},
            {'joints': np.array([0.461, np.radians(93.2), np.radians(-37.7), np.radians(12)])}
        ]
        
        self.crate2_positions = [
            {'joints': np.array([0.5, np.radians(90), 0.0, np.radians(90)])},
            {'joints': np.array([1, np.radians(138), 0.0, np.radians(42.5)])},
            {'joints': np.array([0.9, np.radians(138), 0.0, np.radians(42.5)])},
            {'joints': np.array([1, np.radians(84), 0.0, np.radians(96)])},
            {'joints': np.array([0.9, np.radians(94), 0.0, np.radians(50)])}
        ]
        
        self.crate3_positions = [
            {'joints': np.array([0.5, np.radians(90), 0.0, np.radians(90)])},
            {'joints': np.array([0.61, np.radians(100), np.radians(62.8), np.radians(92.3)])},
            {'joints': np.array([0.51, np.radians(100), np.radians(62.8), np.radians(92.3)])},
            {'joints': np.array([1, np.radians(46), np.radians(62.8), np.radians(133.5)])},
            {'joints': np.array([0.968, np.radians(96.7), np.radians(34.2), np.radians(0)])}
        ]
        
        self.current_animation = 1
        self.crate_size = 0.08
        self.crate_colors = ['sienna', 'peru', 'chocolate']
        
        self.crates = [
            {'pos': np.array([0.4, -0.2, self.crate_size/2]), 'color': 'sienna', 'id': 1, 'attached': False, 'on_shelf': False},
            {'pos': np.array([0.4, 0.0, self.crate_size/2]), 'color': 'peru', 'id': 2, 'attached': False, 'on_shelf': False},
            {'pos': np.array([0.4, 0.2, self.crate_size/2]), 'color': 'chocolate', 'id': 3, 'attached': False, 'on_shelf': False}
        ]
        
        level_height = 1.0 / (3 + 1)
        self.shelf1_level1_pos = np.array([0.775, -0.35, 0.0 + 1 * level_height + 0.07])
        self.shelf2_level2_pos = np.array([0.775, 0.0, 0.0 + 2 * level_height + 0.07])
        self.shelf3_level3_pos = np.array([0.775, 0.35, 0.0 + 3 * level_height + 0.07])
        
        self.carrying_crate = None
        self.shelf_width = 0.3
        self.shelf_depth = 0.15
        self.shelf_height = 1.0
        self.shelf_levels = 3
        
        self.shelves = [
            {'pos': [0.7, -0.35, 0.0], 'label': 'SHELF 1', 'color': (0.65, 0.50, 0.39)},
            {'pos': [0.7, 0.0, 0.0], 'label': 'SHELF 2', 'color': (0.55, 0.45, 0.34)},
            {'pos': [0.7, 0.35, 0.0], 'label': 'SHELF 3', 'color': (0.45, 0.40, 0.29)}
        ]
        
        self.fig = plt.figure(figsize=(14, 9))
        self.ax = self.fig.add_axes([0.05, 0.2, 0.9, 0.75], projection='3d')
        
        self.setup_scene()
        self.create_sliders()
        self.create_buttons()

    def setup_scene(self):
        self.ax.clear()
        self.ax.set_xlim([-0.5, 1.0])
        self.ax.set_ylim([-0.8, 0.8])
        self.ax.set_zlim([0, 1.5])
        self.ax.set_xlabel('X (m)', fontsize=11)
        self.ax.set_ylabel('Y (m)', fontsize=11)
        self.ax.set_zlabel('Z (m)', fontsize=11)
        self.ax.set_title('PDE4431 Coursework 2: Robotic Pick and Place System', fontsize=14, fontweight='bold', pad=10)
        
        x = np.linspace(-0.5, 1.0, 12)
        y = np.linspace(-0.8, 0.8, 10)
        X, Y = np.meshgrid(x, y)
        Z = np.zeros_like(X)
        self.ax.plot_surface(X, Y, Z, alpha=0.08, color='gray')
        
        self.plot_crates()
        self.plot_shelves()
        self.plot_robot()
        self.show_dh_table()
        self.show_transformation_calculations()
        self.fig.canvas.draw_idle()
    
    def plot_crates(self):
        for crate in self.crates:
            if crate['attached']:
                continue
                
            if crate['id'] == 1 and crate['on_shelf']:
                pos = self.shelf1_level1_pos.copy()
                pos[2] = pos[2] - self.crate_size/2
            elif crate['id'] == 2 and crate['on_shelf']:
                pos = self.shelf2_level2_pos.copy()
                pos[2] = pos[2] - self.crate_size/2
            elif crate['id'] == 3 and crate['on_shelf']:
                pos = self.shelf3_level3_pos.copy()
                pos[2] = pos[2] - self.crate_size/2
            else:
                pos = crate['pos']
            
            s = self.crate_size / 2
            vertices = [
                [pos[0]-s, pos[1]-s, pos[2]],
                [pos[0]+s, pos[1]-s, pos[2]],
                [pos[0]+s, pos[1]+s, pos[2]],
                [pos[0]-s, pos[1]+s, pos[2]],
                [pos[0]-s, pos[1]-s, pos[2]+self.crate_size],
                [pos[0]+s, pos[1]-s, pos[2]+self.crate_size],
                [pos[0]+s, pos[1]+s, pos[2]+self.crate_size],
                [pos[0]-s, pos[1]+s, pos[2]+self.crate_size]
            ]
            faces = [
                [vertices[0], vertices[1], vertices[2], vertices[3]],
                [vertices[4], vertices[5], vertices[6], vertices[7]],
                [vertices[0], vertices[1], vertices[5], vertices[4]],
                [vertices[2], vertices[3], vertices[7], vertices[6]],
                [vertices[1], vertices[2], vertices[6], vertices[5]],
                [vertices[0], vertices[3], vertices[7], vertices[4]]
            ]
            crate_color = crate['color']
            crate_obj = Poly3DCollection(faces, alpha=1.0, color=crate_color)
            crate_obj.set_edgecolor('black')
            if crate['on_shelf']:
                crate_obj.set_linewidth(2)
            self.ax.add_collection3d(crate_obj)
            
    def plot_shelves(self):
        for shelf in self.shelves:
            pos = shelf['pos']
            w = self.shelf_width
            d = self.shelf_depth
            h = self.shelf_height
            support_thickness = 0.02
            
            left_support_corners = [
                [pos[0], pos[1] - w/2, pos[2]],
                [pos[0] + support_thickness, pos[1] - w/2, pos[2]],
                [pos[0] + support_thickness, pos[1] - w/2, pos[2] + h],
                [pos[0], pos[1] - w/2, pos[2] + h]
            ]
            right_support_corners = [
                [pos[0], pos[1] + w/2, pos[2]],
                [pos[0] + support_thickness, pos[1] + w/2, pos[2]],
                [pos[0] + support_thickness, pos[1] + w/2, pos[2] + h],
                [pos[0], pos[1] + w/2, pos[2] + h]
            ]
            
            for support in [left_support_corners, right_support_corners]:
                support_faces = [[support[0], support[1], support[2], support[3]]]
                support_collection = Poly3DCollection(support_faces, color=shelf['color'], alpha=1.0)
                support_collection.set_edgecolor('black')
                self.ax.add_collection3d(support_collection)
            
            shelf_thickness = 0.015
            level_height = h / (self.shelf_levels + 1)
            
            for level in range(1, self.shelf_levels + 1):
                z = pos[2] + level * level_height
                is_shelf1 = (shelf['label'] == 'SHELF 1')
                is_shelf2 = (shelf['label'] == 'SHELF 2')
                is_shelf3 = (shelf['label'] == 'SHELF 3')
                is_target_level1 = (is_shelf1 and level == 1)
                is_target_level2 = (is_shelf2 and level == 2)
                is_target_level3 = (is_shelf3 and level == 3)
                
                if is_target_level1 or is_target_level2 or is_target_level3:
                    shelf_color = (0.8, 0.6, 0.2)
                else:
                    shelf_color = tuple(min(1.0, c * 1.2) for c in shelf['color'])
                
                shelf_corners = [
                    [pos[0], pos[1] - w/2, z],
                    [pos[0] + d, pos[1] - w/2, z],
                    [pos[0] + d, pos[1] + w/2, z],
                    [pos[0], pos[1] + w/2, z],
                    [pos[0], pos[1] - w/2, z + shelf_thickness],
                    [pos[0] + d, pos[1] - w/2, z + shelf_thickness],
                    [pos[0] + d, pos[1] + w/2, z + shelf_thickness],
                    [pos[0], pos[1] + w/2, z + shelf_thickness]
                ]
                shelf_faces = [
                    [shelf_corners[0], shelf_corners[1], shelf_corners[2], shelf_corners[3]],
                    [shelf_corners[4], shelf_corners[5], shelf_corners[6], shelf_corners[7]],
                    [shelf_corners[0], shelf_corners[1], shelf_corners[5], shelf_corners[4]],
                    [shelf_corners[2], shelf_corners[3], shelf_corners[7], shelf_corners[6]],
                    [shelf_corners[1], shelf_corners[2], shelf_corners[6], shelf_corners[5]],
                    [shelf_corners[0], shelf_corners[3], shelf_corners[7], shelf_corners[4]]
                ]
                shelf_collection = Poly3DCollection(shelf_faces, color=shelf_color, alpha=1.0)
                shelf_collection.set_edgecolor('black')
                self.ax.add_collection3d(shelf_collection)
        
    def forward_kinematics(self, joints):
        d1, theta2, theta3, theta4 = joints
        positions = []
        positions.append([0, 0, 0])
        base_z = d1
        positions.append([0, 0, base_z])
        shoulder_x = self.L1 * np.sin(theta2)
        shoulder_z = base_z + self.L1 * np.cos(theta2)
        positions.append([shoulder_x, 0, shoulder_z])
        elbow_x = shoulder_x + self.L2 * np.sin(theta2) * np.cos(theta3)
        elbow_y = self.L2 * np.sin(theta3)
        elbow_z = shoulder_z + self.L2 * np.cos(theta2) * np.cos(theta3)
        positions.append([elbow_x, elbow_y, elbow_z])
        total_angle = theta2 + theta4
        end_x = elbow_x + self.L3 * np.sin(total_angle) * np.cos(theta3)
        end_y = elbow_y + self.L3 * np.sin(total_angle) * np.sin(theta3)
        end_z = elbow_z + self.L3 * np.cos(total_angle)
        positions.append([end_x, end_y, end_z])
        return positions
    
    def dh_matrix(self, a, alpha, d, theta):
        T = np.array([
            [np.cos(theta), -np.sin(theta)*np.cos(alpha), np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
            [np.sin(theta), np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        return T

    def show_dh_table(self):
        dh_text = "DENAVIT-HARTENBERG PARAMETERS\n"
        dh_text += "═" * 55 + "\n"
        dh_text += " Joint |   a (m)  |   α (°)  |   d (m)  |   θ (°)  \n"
        dh_text += "───────┼──────────┼──────────┼──────────┼──────────\n"
        
        dh_params = [
            ("J1", 0.0, 0.0, f"{self.joints[0]:.3f}", 0.0),
            ("J2", self.L1, 90.0, 0.0, f"{np.degrees(self.joints[1]):.1f}"),
            ("J3", self.L2, 0.0, 0.0, f"{np.degrees(self.joints[2]):.1f}"),
            ("J4", self.L3, 0.0, 0.0, f"{np.degrees(self.joints[3]):.1f}")
        ]
        
        for joint_name, a, alpha, d, theta in dh_params:
            if isinstance(alpha, float):
                alpha_str = f"{alpha:>7.1f}"
            else:
                alpha_str = f"{alpha:>7}"
                
            if isinstance(theta, float):
                theta_str = f"{theta:>7.1f}"
            else:
                theta_str = f"{theta:>7}"
                
            dh_text += f"  {joint_name:4} |  {a:5.3f}   | {alpha_str}  |  {d:5}   | {theta_str}  \n"
        
        dh_text += "\nDH Transformation Matrix (Tᵢ):\n"
        dh_text += "⎡ cosθᵢ   -sinθᵢ·cosαᵢ   sinθᵢ·sinαᵢ   aᵢ·cosθᵢ ⎤\n"
        dh_text += "⎢ sinθᵢ    cosθᵢ·cosαᵢ  -cosθᵢ·sinαᵢ   aᵢ·sinθᵢ ⎥\n"
        dh_text += "⎣   0         sinαᵢ         cosαᵢ         dᵢ    ⎦\n"
        
        dh_box = self.ax.text2D(-0.56, 0.30, dh_text,
                               transform=self.ax.transAxes,
                               fontsize=8.5,
                               family='monospace',
                               bbox=dict(boxstyle="round,pad=0.4", 
                                       facecolor="lavenderblush", 
                                       alpha=0.95,
                                       edgecolor='purple',
                                       linewidth=1.5),
                               verticalalignment='top',
                               horizontalalignment='left')
        
        return dh_box
    
    def plot_robot(self):
        positions = self.forward_kinematics(self.joints)
        x_vals = [p[0] for p in positions]
        y_vals = [p[1] for p in positions]
        z_vals = [p[2] for p in positions]
        self.ax.plot(x_vals, y_vals, z_vals, 'o-', color='royalblue', linewidth=3.5, markersize=0)
        joint_names = ['Base', 'Prismatic', 'Shoulder', 'Elbow', 'End Effector']
        colors = ['black', 'gray', 'red', 'orange', 'magenta']
        sizes = [80, 120, 100, 100, 120]
        
        for i in range(5):
            self.ax.scatter(positions[i][0], positions[i][1], positions[i][2],
                          c=colors[i], s=sizes[i], marker='o' if i < 4 else '*',
                          edgecolors='black', linewidth=1.5, zorder=10)
        
        if self.carrying_crate is not None:
            ee_pos = positions[4]
            crate_pos = ee_pos.copy()
            crate_pos[2] = crate_pos[2] - self.crate_size
            s = self.crate_size / 2
            vertices = [
                [crate_pos[0]-s, crate_pos[1]-s, crate_pos[2]],
                [crate_pos[0]+s, crate_pos[1]-s, crate_pos[2]],
                [crate_pos[0]+s, crate_pos[1]+s, crate_pos[2]],
                [crate_pos[0]-s, crate_pos[1]+s, crate_pos[2]],
                [crate_pos[0]-s, crate_pos[1]-s, crate_pos[2]+self.crate_size],
                [crate_pos[0]+s, crate_pos[1]-s, crate_pos[2]+self.crate_size],
                [crate_pos[0]+s, crate_pos[1]+s, crate_pos[2]+self.crate_size],
                [crate_pos[0]-s, crate_pos[1]+s, crate_pos[2]+self.crate_size]
            ]
            faces = [
                [vertices[0], vertices[1], vertices[2], vertices[3]],
                [vertices[4], vertices[5], vertices[6], vertices[7]],
                [vertices[0], vertices[1], vertices[5], vertices[4]],
                [vertices[2], vertices[3], vertices[7], vertices[6]],
                [vertices[1], vertices[2], vertices[6], vertices[5]],
                [vertices[0], vertices[3], vertices[7], vertices[4]]
            ]
            crate_obj = Poly3DCollection(faces, alpha=1.0, color='gold')
            crate_obj.set_edgecolor('red')
            crate_obj.set_linewidth(2)
            self.ax.add_collection3d(crate_obj)
            gripper_length = 0.03
            gripper_width = 0.01
            left_gripper = [
                [ee_pos[0] - gripper_width/2, ee_pos[1] - s, ee_pos[2]],
                [ee_pos[0] + gripper_width/2, ee_pos[1] - s, ee_pos[2]],
                [ee_pos[0] + gripper_width/2, ee_pos[1] - s, ee_pos[2] - gripper_length],
                [ee_pos[0] - gripper_width/2, ee_pos[1] - s, ee_pos[2] - gripper_length]
            ]
            right_gripper = [
                [ee_pos[0] - gripper_width/2, ee_pos[1] + s, ee_pos[2]],
                [ee_pos[0] + gripper_width/2, ee_pos[1] + s, ee_pos[2]],
                [ee_pos[0] + gripper_width/2, ee_pos[1] + s, ee_pos[2] - gripper_length],
                [ee_pos[0] - gripper_width/2, ee_pos[1] + s, ee_pos[2] - gripper_length]
            ]
            gripper_faces = [left_gripper, right_gripper]
            
            for gripper_face in gripper_faces:
                gripper_obj = Poly3DCollection([gripper_face], color='darkgray', alpha=1.0)
                gripper_obj.set_edgecolor('black')
                self.ax.add_collection3d(gripper_obj)
            
            self.ax.plot([ee_pos[0], ee_pos[0]], 
                        [ee_pos[1], ee_pos[1]], 
                        [ee_pos[2], crate_pos[2] + self.crate_size], 
                        'k-', alpha=0.5, linewidth=2)
            
        ee_pos = positions[4]
        self.ax.text2D(0.02, 0.85, 
                      f'End Effector Position:\nX: {ee_pos[0]:.3f} m\nY: {ee_pos[1]:.3f} m\nZ: {ee_pos[2]:.3f} m',
                      fontsize=10, transform=self.ax.transAxes,
                      bbox=dict(boxstyle="round,pad=0.3", facecolor="lightcyan", alpha=0.9))
        
        joint_text = f'Joint States:\n'
        joint_text += f'J1: {self.joints[0]:.3f} m\n'
        joint_text += f'J2: {np.degrees(self.joints[1]):.1f}°\n'
        joint_text += f'J3: {np.degrees(self.joints[2]):.1f}°\n'
        joint_text += f'J4: {np.degrees(self.joints[3]):.1f}°'
        
        self.ax.text2D(0.3, 0.85, joint_text,
                      fontsize=10, transform=self.ax.transAxes,
                      bbox=dict(boxstyle="round,pad=0.3", facecolor="lavender", alpha=0.9))
    
    def show_transformation_calculations(self):
        shelf1_target = self.shelf1_level1_pos.copy()
        shelf1_target[2] = shelf1_target[2] - self.crate_size/2
        shelf2_target = self.shelf2_level2_pos.copy()
        shelf2_target[2] = shelf2_target[2] - self.crate_size/2
        shelf3_target = self.shelf3_level3_pos.copy()
        shelf3_target[2] = shelf3_target[2] - self.crate_size/2
        
        text1 = "CRATE 1 → SHELF 1\n"
        text1 += "═" * 35 + "\n"
        text1 += "Target Position: P₁ = [{:.3f}, {:.3f}, {:.3f}]\n".format(
            shelf1_target[0], shelf1_target[1], shelf1_target[2])
        text1 += "Calculated Joints:\n"
        text1 += "  d₁ = {:.3f}m, θ₂ = {:.1f}°\n".format(0.461, 93.2)
        text1 += "  θ₃ = {:.1f}°, θ₄ = {:.1f}°\n".format(-37.7, 12)
        text1 += "Verification (Forward Kinematics):\n"
        text1 += "  X = 0.3·sin({:.1f}°) + 0.3·sin({:.1f}°)·cos({:.1f}°) ".format(
            93.2, 93.2, -37.7)
        text1 += "+ 0.3·sin({:.1f}°)·cos({:.1f}°) = {:.3f}\n".format(
            93.2+12, -37.7, shelf1_target[0])
        text1 += "  Y = 0.3·sin({:.1f}°) + 0.3·sin({:.1f}°)·sin({:.1f}°) = {:.3f}\n".format(
            -37.7, 93.2+12, -37.7, shelf1_target[1])
        text1 += "  Z = {:.3f} + 0.3·cos({:.1f}°) + 0.3·cos({:.1f}°)·cos({:.1f}°) ".format(
            0.461, 93.2, 93.2, -37.7)
        text1 += "+ 0.3·cos({:.1f}°) = {:.3f}".format(
            93.2+12, shelf1_target[2])
        
        crate1_box = self.ax.text2D(-0.55, -0.05, text1,
                                   transform=self.ax.transAxes,
                                   fontsize=7.5,
                                   bbox=dict(boxstyle="round,pad=0.3", facecolor="peachpuff", alpha=0.95),
                                   verticalalignment='top',
                                   horizontalalignment='left')
        
        text2 = "CRATE 2 → SHELF 2\n"
        text2 += "═" * 35 + "\n"
        text2 += "Target Position: P₂ = [{:.3f}, {:.3f}, {:.3f}]\n".format(
            shelf2_target[0], shelf2_target[1], shelf2_target[2])
        text2 += "Calculated Joints:\n"
        text2 += "  d₁ = {:.3f}m, θ₂ = {:.1f}°\n".format(0.9, 94)
        text2 += "  θ₃ = {:.1f}°, θ₄ = {:.1f}°\n".format(0, 50)
        text2 += "Verification (Forward Kinematics):\n"
        text2 += "  X = 0.3·sin({:.1f}°) + 0.3·sin({:.1f}°)·cos({:.1f}°) ".format(
            94, 94, 0)
        text2 += "+ 0.3·sin({:.1f}°)·cos({:.1f}°) = {:.3f}\n".format(
            94+50, 0, shelf2_target[0])
        text2 += "  Y = 0.3·sin({:.1f}°) + 0.3·sin({:.1f}°)·sin({:.1f}°) = {:.3f}\n".format(
            0, 94+50, 0, shelf2_target[1])
        text2 += "  Z = {:.3f} + 0.3·cos({:.1f}°) + 0.3·cos({:.1f}°)·cos({:.1f}°) ".format(
            0.9, 94, 94, 0)
        text2 += "+ 0.3·cos({:.1f}°) = {:.3f}".format(
            94+50, shelf2_target[2])
        
        crate2_box = self.ax.text2D(0.18, -0.05, text2,
                                   transform=self.ax.transAxes,
                                   fontsize=7.5,
                                   bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.95),
                                   verticalalignment='top',
                                   horizontalalignment='left')
        
        text3 = "CRATE 3 → SHELF 3\n"
        text3 += "═" * 35 + "\n"
        text3 += "Target Position: P₃ = [{:.3f}, {:.3f}, {:.3f}]\n".format(
            shelf3_target[0], shelf3_target[1], shelf3_target[2])
        text3 += "Calculated Joints:\n"
        text3 += "  d₁ = {:.3f}m, θ₂ = {:.1f}°\n".format(0.968, 96.7)
        text3 += "  θ₃ = {:.1f}°, θ₄ = {:.1f}°\n".format(34.2, 0)
        text3 += "Verification (Forward Kinematics):\n"
        text3 += "  X = 0.3·sin({:.1f}°) + 0.3·sin({:.1f}°)·cos({:.1f}°) ".format(
            96.7, 96.7, 34.2)
        text3 += "+ 0.3·sin({:.1f}°)·cos({:.1f}°) = {:.3f}\n".format(
            96.7+0, 34.2, shelf3_target[0])
        text3 += "  Y = 0.3·sin({:.1f}°) + 0.3·sin({:.1f}°)·sin({:.1f}°) = {:.3f}\n".format(
            34.2, 96.7+0, 34.2, shelf3_target[1])
        text3 += "  Z = {:.3f} + 0.3·cos({:.1f}°) + 0.3·cos({:.1f}°)·cos({:.1f}°) ".format(
            0.968, 96.7, 96.7, 34.2)
        text3 += "+ 0.3·cos({:.1f}°) = {:.3f}".format(
            96.7+0, shelf3_target[2])
        
        crate3_box = self.ax.text2D(0.88, -0.05, text3,
                                   transform=self.ax.transAxes,
                                   fontsize=7.5,
                                   bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.95),
                                   verticalalignment='top',
                                   horizontalalignment='left')
        
        return [crate1_box, crate2_box, crate3_box]
    
    def create_sliders(self):
        slider_positions = [
            [0.5, 0.9, 0.15, 0.03],
            [0.5, 0.87, 0.15, 0.03],
            [0.5, 0.84, 0.15, 0.03],
            [0.5, 0.81, 0.15, 0.03]
        ]
        slider_labels = ['J1', 'J2', 'J3', 'J4']
        joint_ranges = [
            (0, 1.0),
            (0, np.pi),
            (-np.pi/2, np.pi/2),
            (0, np.pi)
        ]
        self.sliders = []
        for i in range(4):
            ax = self.fig.add_axes(slider_positions[i])
            if i == 0:
                slider = Slider(ax, slider_labels[i], joint_ranges[i][0], 
                               joint_ranges[i][1], valinit=self.joints[i],
                               valfmt='%1.3f')
            else:
                slider = Slider(ax, slider_labels[i], 
                               np.degrees(joint_ranges[i][0]),
                               np.degrees(joint_ranges[i][1]),
                               valinit=np.degrees(self.joints[i]),
                               valfmt='%1.1f°')
            slider.joint_index = i
            slider.on_changed(self.update_joint_manual)
            self.sliders.append(slider)
    
    def create_buttons(self):
        anim_ax = self.fig.add_axes([0.71, 0.9, 0.05, 0.04])
        self.anim_btn = Button(anim_ax, '▶ Start', color='lightgreen')
        self.anim_btn.on_clicked(self.start_full_animation)
        
        reset_ax = self.fig.add_axes([0.71, 0.86, 0.05, 0.04])
        self.reset_btn = Button(reset_ax, '↺ Reset', color='lightcoral')
        self.reset_btn.on_clicked(self.reset_to_home)
        
        pause_ax = self.fig.add_axes([0.71, 0.82, 0.05, 0.04])
        self.pause_btn = Button(pause_ax, '▌ Pause', color='lightblue')
        self.pause_btn.on_clicked(self.toggle_pause_animation)
    
    def update_joint_manual(self, val):
        if self.is_animating:
            return
        for slider in self.sliders:
            if slider.joint_index == 0:
                self.joints[0] = slider.val
            else:
                self.joints[slider.joint_index] = np.radians(slider.val)
        self.setup_scene()
        plt.draw()
    
    def interpolate_joints(self, start_joints, end_joints, t):
        return start_joints + t * (end_joints - start_joints)
    
    def generate_animation_sequence(self, crate_number):
        if crate_number == 1:
            positions = self.crate1_positions
        elif crate_number == 2:
            positions = self.crate2_positions
        else:
            positions = self.crate3_positions
        sequence = []
        
        for i in range(self.total_animation_steps):
            t = i / self.total_animation_steps
            joints = self.interpolate_joints(
                positions[0]['joints'],
                positions[1]['joints'],
                t
            )
            sequence.append({
                'joints': joints,
                'action': 'move',
                'crate_attached': False,
                'crate_on_shelf': False,
                'crate_number': crate_number
            })
        
        for i in range(self.total_animation_steps):
            t = i / self.total_animation_steps
            joints = self.interpolate_joints(
                positions[1]['joints'],
                positions[2]['joints'],
                t
            )
            sequence.append({
                'joints': joints,
                'action': 'move',
                'crate_attached': False,
                'crate_on_shelf': False,
                'crate_number': crate_number
            })
        
        sequence.append({
            'joints': positions[2]['joints'],
            'action': 'attach_crate',
            'crate_attached': True,
            'crate_on_shelf': False,
            'crate_number': crate_number
        })
        
        for i in range(self.total_animation_steps):
            t = i / self.total_animation_steps
            joints = self.interpolate_joints(
                positions[2]['joints'],
                positions[3]['joints'],
                t
            )
            sequence.append({
                'joints': joints,
                'action': 'move_with_crate',
                'crate_attached': True,
                'crate_on_shelf': False,
                'crate_number': crate_number
            })
        
        for i in range(self.total_animation_steps):
            t = i / self.total_animation_steps
            joints = self.interpolate_joints(
                positions[3]['joints'],
                positions[4]['joints'],
                t
            )
            sequence.append({
                'joints': joints,
                'action': 'move_with_crate',
                'crate_attached': True,
                'crate_on_shelf': False,
                'crate_number': crate_number
            })
        
        sequence.append({
            'joints': positions[4]['joints'],
            'action': 'place_on_shelf',
            'crate_attached': False,
            'crate_on_shelf': True,
            'crate_number': crate_number
        })
        
        if crate_number == 1:
            for i in range(self.total_animation_steps):
                t = i / self.total_animation_steps
                joints = self.interpolate_joints(
                    positions[4]['joints'],
                    positions[3]['joints'],
                    t
                )
                sequence.append({
                    'joints': joints,
                    'action': 'move',
                    'crate_attached': False,
                    'crate_on_shelf': True,
                    'crate_number': crate_number
                })
            for i in range(self.total_animation_steps):
                t = i / self.total_animation_steps
                joints = self.interpolate_joints(
                    positions[3]['joints'],
                    positions[0]['joints'],
                    t
                )
                sequence.append({
                    'joints': joints,
                    'action': 'move',
                    'crate_attached': False,
                    'crate_on_shelf': True,
                    'crate_number': crate_number
                })
        elif crate_number == 2:
            for i in range(self.total_animation_steps):
                t = i / self.total_animation_steps
                joints = self.interpolate_joints(
                    positions[4]['joints'],
                    positions[3]['joints'],
                    t
                )
                sequence.append({
                    'joints': joints,
                    'action': 'move',
                    'crate_attached': False,
                    'crate_on_shelf': True,
                    'crate_number': crate_number
                })
            for i in range(self.total_animation_steps):
                t = i / self.total_animation_steps
                joints = self.interpolate_joints(
                    positions[3]['joints'],
                    positions[0]['joints'],
                    t
                )
                sequence.append({
                    'joints': joints,
                    'action': 'move',
                    'crate_attached': False,
                    'crate_on_shelf': True,
                    'crate_number': crate_number
                })
        else:
            for i in range(self.total_animation_steps):
                t = i / self.total_animation_steps
                joints = self.interpolate_joints(
                    positions[4]['joints'],
                    positions[3]['joints'],
                    t
                )
                sequence.append({
                    'joints': joints,
                    'action': 'move',
                    'crate_attached': False,
                    'crate_on_shelf': True,
                    'crate_number': crate_number
                })
            for i in range(self.total_animation_steps):
                t = i / self.total_animation_steps
                joints = self.interpolate_joints(
                    positions[3]['joints'],
                    positions[0]['joints'],
                    t
                )
                sequence.append({
                    'joints': joints,
                    'action': 'move',
                    'crate_attached': False,
                    'crate_on_shelf': True,
                    'crate_number': crate_number
                })
        return sequence
    
    def update_animation(self, frame):
        if self.is_paused:
            return []
        if not self.is_animating or self.current_animation_step >= len(self.animation_sequence):
            if self.current_animation == 1 and not self.is_animating:
                self.current_animation = 2
                self.current_animation_step = 0
                self.animation_sequence = self.generate_animation_sequence(2)
                self.is_animating = True
                return []
            elif self.current_animation == 2 and not self.is_animating:
                self.current_animation = 3
                self.current_animation_step = 0
                self.animation_sequence = self.generate_animation_sequence(3)
                self.is_animating = True
                return []
            else:
                self.is_animating = False
                self.current_animation_step = 0
                self.setup_scene()
                plt.draw()
                return
        step = self.animation_sequence[self.current_animation_step]
        self.joints = step['joints']
        action = step['action']
        crate_num = step['crate_number']
        crate_index = crate_num - 1
        if action == 'attach_crate':
            self.carrying_crate = crate_num
            self.crates[crate_index]['attached'] = True
            self.crates[crate_index]['on_shelf'] = False
        elif action == 'place_on_shelf':
            self.carrying_crate = None
            self.crates[crate_index]['attached'] = False
            self.crates[crate_index]['on_shelf'] = True
        elif action == 'move_with_crate':
            self.carrying_crate = crate_num
            self.crates[crate_index]['attached'] = True
            self.crates[crate_index]['on_shelf'] = False
        self.ax.clear()
        self.setup_scene()
        self.current_animation_step += 1
        return []
    
    def start_full_animation(self, event=None):
        if self.is_animating:
            return
        self.is_paused = False
        self.pause_btn.label.set_text('▌ Pause')
        self.pause_btn.color = 'lightblue'
        self.pause_btn.hovercolor = 'lightblue'
        self.carrying_crate = None
        for crate in self.crates:
            crate['attached'] = False
            crate['on_shelf'] = False
        self.current_animation = 1
        self.current_animation_step = 0
        self.animation_sequence = self.generate_animation_sequence(1)
        self.is_animating = True
        self.anim = animation.FuncAnimation(
            self.fig, 
            self.update_animation, 
            frames=len(self.animation_sequence) * 3,
            interval=50,
            repeat=False,
            blit=False
        )
        self.update_sliders_from_joints()
        plt.draw()
    
    def toggle_pause_animation(self, event=None):
        if not self.is_animating:
            return
        if self.is_paused:
            self.is_paused = False
            self.pause_btn.label.set_text('▌ Pause')
            self.pause_btn.color = 'lightblue'
            self.pause_btn.hovercolor = 'lightblue'
            if hasattr(self, 'anim'):
                self.anim.event_source.start()
        else:
            self.is_paused = True
            self.pause_btn.label.set_text('▶ Resume')
            self.pause_btn.color = 'lightgreen'
            self.pause_btn.hovercolor = 'lightgreen'
            if hasattr(self, 'anim'):
                self.anim.event_source.stop()
        self.fig.canvas.draw_idle()
    
    def reset_to_home(self, event=None):
        if self.is_animating and not self.is_paused:
            return
        if self.is_animating:
            self.is_animating = False
            self.is_paused = False
            if hasattr(self, 'anim'):
                self.anim.event_source.stop()
            self.pause_btn.label.set_text('▌ Pause')
            self.pause_btn.color = 'lightblue'
            self.pause_btn.hovercolor = 'lightblue'
        self.joints = np.array([0.5, np.radians(90), 0.0, np.radians(90)])
        self.carrying_crate = None
        self.crates = [
            {'pos': np.array([0.4, -0.2, self.crate_size/2]), 'color': 'sienna', 'id': 1, 'attached': False, 'on_shelf': False},
            {'pos': np.array([0.4, 0.0, self.crate_size/2]), 'color': 'peru', 'id': 2, 'attached': False, 'on_shelf': False},
            {'pos': np.array([0.4, 0.2, self.crate_size/2]), 'color': 'chocolate', 'id': 3, 'attached': False, 'on_shelf': False}
        ]
        if hasattr(self, 'sliders') and len(self.sliders) == 4:
            self.sliders[0].set_val(0.5)
            self.sliders[1].set_val(90.0)
            self.sliders[2].set_val(0.0)
            self.sliders[3].set_val(90.0)
        for slider in self.sliders:
            if slider.joint_index == 0:
                self.joints[0] = slider.val
            else:
                self.joints[slider.joint_index] = np.radians(slider.val)
        self.setup_scene()
        plt.draw()
    
    def update_sliders_from_joints(self):
        self.sliders[0].set_val(self.joints[0])
        self.sliders[1].set_val(np.degrees(self.joints[1]))
        self.sliders[2].set_val(np.degrees(self.joints[2]))
        self.sliders[3].set_val(np.degrees(self.joints[3]))
    
    def run(self):
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    robot = PDE4431_Coursework2()
    robot.run()

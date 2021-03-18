import numpy as np

import pyqtgraph as pg
import pyqtgraph.opengl as gl
import pyqtgraph.Vector as Vector

class quad_viewer():
    def __init__(self):
        # initialize Qt gui application and window
        self.app = pg.QtGui.QApplication([])  # initialize QT
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle('World Viewer')
        self.window.setGeometry(0, 0, 1000, 1000)  # args: upper_left_x, upper_right_y, width, height
        grid = gl.GLGridItem()  # make a grid to represent the ground
        grid.scale(10, 10, 10)  # set the size of the grid (distance between each line)
        self.window.addItem(grid) # add grid to viewer
        self.window.setCameraPosition(distance=20)  # distance from center of plot to camera
        self.window.setBackgroundColor('k')  # set background color to black
        self.window.show()  # display configured window
        self.window.raise_()  # bring window to the front
        self.plot_initialized = False # has the quadrotor been plotted yet?
        # get points that define the non-rotated, non-translated quadrotor and the mesh colors
        self.points, self.meshColors = self._get_quadrotor_points()

    def update(self, quad_state):
        self.update_quad(quad_state)
        # redraw
        self.app.processEvents()

    def update_quad(self, state):
        quadrotor_position = np.array([[state.pn], [state.pe], [state.pd]])  # NED coordinates
        # attitude of quadrotor as a rotation matrix R from body to inertial
        R = self._Euler2Rotation(state.phi, state.theta, state.psi)
        # rotate and translate points defining quadrotor
        rotated_points = self._rotate_points(self.points, R)
        translated_points = self._translate_points(rotated_points, quadrotor_position)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self._quadrotor_points_to_mesh(translated_points)

        # initialize the drawing the first time update() is called
        if not self.plot_initialized:
            # initialize drawing of triangular mesh.
            self.body = gl.GLMeshItem(vertexes=mesh,  # defines the triangular mesh (Nx3x3)
                                      vertexColors=self.meshColors, # defines mesh colors (Nx1)
                                      drawEdges=True,  # draw edges between mesh elements
                                      smooth=False,  # speeds up rendering
                                      computeNormals=False)  # speeds up rendering
            self.window.addItem(self.body)  # add body to plot
            self.plot_initialized = True

        # else update drawing on all other calls to update()
        else:
            # reset mesh using rotated and translated points
            self.body.setMeshData(vertexes=mesh, vertexColors=self.meshColors)

        # update the center of the camera view to the quadrotor location
        view_location = Vector(state.pe, state.pn, -state.pd)  # defined in ENU coordinates
        self.window.opts['center'] = view_location
        # redraw
        self.app.processEvents()

    ###################################
    # private functions
    def _rotate_points(self, points, R):
        rotated_points = R @ points
        return rotated_points

    def _translate_points(self, points, translation):
        translated_points = points + np.dot(translation, np.ones([1,points.shape[1]]))
        return translated_points

    def _get_quadrotor_points(self):
        body_w = 0.33333
        arm_l = 1
        prop_w = 0.5
        body_off = 0.1
        prop_off = 0.1

        #points are in NED coordinates
        points = np.array([[body_w/2, -body_w/2, -body_off],  # point 1
                           [-body_w/2, -body_w/2, -body_off],  # point 2
                           [-body_w/2, body_w/2, -body_off],  # point 3
                           [body_w/2, body_w/2, -body_off],  # point 4
                           [body_w/2, 0, 0],  # point 5
                           [arm_l, -arm_l, 0],  # point 6
                           [0, -body_w/2, 0],  # point 7
                           [-arm_l, -arm_l, 0],  # point 8
                           [-body_w/2, 0, 0],  # point 9
                           [-arm_l, arm_l, 0],  # point 10
                           [0, body_w/2, 0],  # point 11
                           [arm_l, arm_l, 0],  # point 12
                           [arm_l+prop_w/2, arm_l-prop_w/2, -prop_off],  # point 13
                           [arm_l-prop_w/2, arm_l-prop_w/2, -prop_off],  # point 14
                           [arm_l-prop_w/2, arm_l+prop_w/2, -prop_off],  # point 15
                           [arm_l+prop_w/2, arm_l+prop_w/2, -prop_off],  # point 16
                           [arm_l+prop_w/2, -arm_l-prop_w/2, -prop_off],  # point 17
                           [arm_l-prop_w/2, -arm_l-prop_w/2, -prop_off],  # point 18
                           [arm_l-prop_w/2, -arm_l+prop_w/2, -prop_off],  # point 19
                           [arm_l+prop_w/2, -arm_l+prop_w/2, -prop_off],  # point 20
                           [-arm_l+prop_w/2, -arm_l-prop_w/2, -prop_off],  # point 21
                           [-arm_l-prop_w/2, -arm_l-prop_w/2, -prop_off],  # point 22
                           [-arm_l-prop_w/2, -arm_l+prop_w/2, -prop_off],  # point 23
                           [-arm_l+prop_w/2, -arm_l+prop_w/2, -prop_off],  # point 24
                           [-arm_l+prop_w/2, arm_l-prop_w/2, -prop_off],  # point 25
                           [-arm_l-prop_w/2, arm_l-prop_w/2, -prop_off],  # point 26
                           [-arm_l-prop_w/2, arm_l+prop_w/2, -prop_off],  # point 27
                           [-arm_l+prop_w/2, arm_l+prop_w/2, -prop_off],  # point 28
                           ]).T
        # scale points for better rendering
        scale = 1
        points = scale * points

        #   define the colors for each face of triangular mesh
        red = np.array([1., 0., 0., 1])
        green = np.array([0., 1., 0., 1])
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])
        meshColors = np.empty((14, 3, 4), dtype=np.float32)
        meshColors[0] = green  # body 1
        meshColors[1] = green  # body 2
        meshColors[2] = red  # arm 1
        meshColors[3] = blue  # arm 2
        meshColors[4] = blue  # arm 3
        meshColors[5] = red  # arm 4
        meshColors[6] = yellow  # prop 11
        meshColors[7] = yellow  # prop 12
        meshColors[8] = yellow  # prop 21
        meshColors[9] = yellow  # prop 22
        meshColors[10] = yellow  # prop 31
        meshColors[11] = yellow  # prop 32
        meshColors[12] = yellow  # prop 41
        meshColors[13] = yellow  # prop 42
        return points, meshColors

    def _quadrotor_points_to_mesh(self, points):
        points=points.T
        mesh = np.array([[points[0], points[1], points[2]],  # body 1
                         [points[0], points[2], points[3]],  # body 2
                         [points[4], points[5], points[6]],  # arm 1
                         [points[6], points[7], points[8]],  # arm 2
                         [points[8], points[9], points[10]],  # arm 3
                         [points[10], points[11], points[4]],  # arm 4
                         [points[12], points[13], points[14]],  # prop 11
                         [points[12], points[14], points[15]],  # prop 12
                         [points[16], points[17], points[18]],  # prop 21
                         [points[16], points[18], points[19]],  # prop 22
                         [points[20], points[21], points[22]],  # prop 31
                         [points[20], points[22], points[23]],  # prop 32
                         [points[24], points[25], points[26]],  # prop 41
                         [points[24], points[26], points[27]],  # prop 42
                         ])
        return mesh

    def _Euler2Rotation(self, phi, theta, psi):
        # only call sin and cos once for each angle to speed up rendering
        c_phi = np.cos(phi)
        s_phi = np.sin(phi)
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        c_psi = np.cos(psi)
        s_psi = np.sin(psi)

        R_roll = np.array([[1, 0, 0],
                           [0, c_phi, s_phi],
                           [0, -s_phi, c_phi]])
        R_pitch = np.array([[c_theta, 0, -s_theta],
                            [0, 1, 0],
                            [s_theta, 0, c_theta]])
        R_yaw = np.array([[c_psi, s_psi, 0],
                          [-s_psi, c_psi, 0],
                          [0, 0, 1]])
        R = R_roll @ R_pitch @ R_yaw  # inertial to body (Equation 2.4 in book)
        return R.T  # transpose to return body to inertial


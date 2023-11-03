import numpy as np
import cv2

T_left_ev = np.array([[ 0.9999407352369797  , 0.009183655542749752,  0.005846920950435052,  0.0005085820608404798], 
                [-0.009131364645448854, 0.9999186289230431  , -0.008908070070089353, -0.04081979450823404],
                [-0.005928253827254812, 0.008854151768176144,  0.9999432282899994  , -0.0140781304960408],
                [0, 0,  0,  1]])

T_left_imu = np.array([[0.017248643674008135, -0.9998037138739959, 0.009747718459772736, 0.07733078169916466],
                    [0.012834636469124028, -0.009526963092989282, -0.999872246379971, -0.016637889364465353],
                    [0.9997688514842376, 0.017371548520172697, 0.01266779001636642, -0.14481844113148515],
                    [0.0, 0.0, 0.0, 1.0]])

print(np.linalg.inv(T_left_ev) * T_left_imu)

# event cam to world
# T_w_ev = np.array([[0.99990929,  0.01212169,  0.00587235, -0.12212969],
#                    [-0.01283288,  0.98978362, 0.14199895, -0.08461687],
#                    [-0.00409109, -0.14206143,  0.98984939, 0.47258243],
#                    [0, 0, 0, 1]])
# # rgb cam to world
# T_w_rgb = np. array([[0.99994815,  0.0095483, -0.00353805, -0.15374702],
#                      [-0.00906599,  0.99301822,  0.11761213, -0.14551716],
#                      [0.00463635, -0.11757396,  0.99305331, 0.47987204],
#                      [0, 0, 0, 1]])

# R_ev = T_w_ev[0:3, 0:3]
# t_ev = T_w_ev[0:3, 3:4]
# R_rgb = T_w_rgb[0:3, 0:3]
# t_rgb = T_w_rgb[0:3, 3:4]

# R_ev_rgb = R_ev.T @ R_rgb
# t_ev_rgb = R_ev.T @ (t_rgb - t_ev)

# # rgb cam to event cam
# T_ev_rgb = np.zeros([4, 4])
# T_ev_rgb[0:3, 0:3] = R_ev_rgb
# T_ev_rgb[0:3, 3:4] = t_ev_rgb
# T_ev_rgb[3, 3] = 1


# T_rgb_depth = np.array([
#     [0.999999, 0.000504241, -0.00124884, -0.0320054],
#     [-0.00037795, 0.995074, 0.0991371, -0.00184325],
#     [0.00129268, -0.0991365, 0.995073, 0.00405237],
#     [0, 0, 0, 1]])

# K_depth_nfov_unbinned_720p = np.array([[504.574, 0,   321.848],
#                                        [0, 504.674, 336.139],
#                                        [0, 0, 1]])


# dist_depth_nfov_unbinned_720p = np.array([5.3633,
#                                           3.35493,
#                                           0.163032,
#                                           -1.08168e-05,
#                                           -1.43868e-05])

# K_depth_nfov_binned_720p = np.array([[252.287, 0, 160.674],
#                                      [0, 252.337, 167.819],
#                                      [0, 0, 1]])

# dist_depth_nfov_binned_720p = np.array([5.3633,
#                                         3.35493,
#                                         0.163032,
#                                         -1.08168e-05,
#                                         -1.43868e-05])

# K_depth_wfov_unbinned_720p = np.array([[504.574, 0,   513.848],
#                                        [0,      504.674, 516.139],
#                                        [0, 0, 1]])

# dist_depth_wfov_unbinned_720p = np.array([5.3633,
#                                           3.35493,
#                                           0.163032,
#                                           -1.08168e-05,
#                                           -1.43868e-05])

# K_depth_wfov_binned_720p = np.array([[252.287, 0,   256.674],
#                                      [0,  252.337, 257.819],
#                                      [0, 0, 1]])

# dist_depth_wfov_binned_720p = np.array([5.3633,
#                                         3.35493,
#                                         0.163032,
#                                         -1.08168e-05,
#                                         -1.43868e-05])

# K_rgb = np.array([[602.965, 0,      640.028],
#                   [0,      602.786, 364.642],
#                   [0,     0,      1]])

# dist_rgb = np.array([0.902495,
#                      -2.96383,
#                      1.60099,
#                      0.000432662,
#                      -0.000477238])

# K_ev = np.array([[1.0472583448973785e+03, 0, 6.6424148637159772e+02],
#                  [0, 1.0471460856502456e+03, 3.5344749919531529e+02],
#                  [0, 0, 1]])

# dist_ev = np.array([- 4.2765623741294628e-01, -5.4924935925829511e-02,
#                     1.5691302194038064e-04, 1.5839076916567842e-03,
#                     1.0466365906696868e+00])
# fs = cv2.FileStorage("mpl_ev_rgbd.yaml", cv2.FILE_STORAGE_WRITE)
# fs.write("T_ev_rgb", T_ev_rgb)
# fs.write("T_rgb_depth", T_ev_rgb)

# fs.write("K_depth_nfov_unbinned_720p", K_depth_nfov_unbinned_720p)
# fs.write("K_depth_nfov_binned_720p", K_depth_nfov_binned_720p)
# fs.write("K_depth_wfov_unbinned_720p", K_depth_wfov_unbinned_720p)
# fs.write("K_depth_wfov_binned_720p", K_depth_wfov_binned_720p)
# fs.write("K_rgb", K_rgb)
# fs.write("K_ev", K_ev)

# fs.write("dist_depth_nfov_unbinned_720p", dist_depth_nfov_unbinned_720p)
# fs.write("dist_depth_nfov_binned_720p", dist_depth_nfov_binned_720p)
# fs.write("dist_depth_wfov_unbinned_720p", dist_depth_wfov_unbinned_720p)
# fs.write("dist_depth_wfov_binned_720p", dist_depth_wfov_binned_720p)
# fs.write("dist_rgb", dist_rgb)
# fs.write("dist_ev", dist_ev)

# fs.release()

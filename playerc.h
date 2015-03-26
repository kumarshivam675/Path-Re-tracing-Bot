/*his program is distributed in the hope that it will be useful,
ut WITHOUT ANY WARRANTY; without even the implied warranty of
ERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
NU General Public License for more details.
NU General Public License for more details.
ou should have received a copy of the GNU General Public License
long with this program; if not, write to the Free Software
oundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
oundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
oundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
oundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
layer - One Hell of a Robot Server
opyright (C) Andrew Howard 2003
  *                      
  *
  *  This library is free software; you can redistribute it and/or
  *  modify it under the terms of the GNU Lesser General Public
  *  License as published by the Free Software Foundation; either
  *  version 2.1 of the License, or (at your option) any later version.
  *
  *  This library is distributed in the hope that it will be useful,
  *  but WITHOUT ANY WARRANTY; without even the implied warranty of
  *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  *  Lesser General Public License for more details.
  *
  *  You should have received a copy of the GNU Lesser General Public
  *  License along with this library; if not, write to the Free Software
  *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
  */

 /***************************************************************************
  * Desc: Player client
  * Author: Andrew Howard
  * Date: 24 Aug 2001
  # CVS: $Id: playerc.h,v 1.118 2005/04/22 04:43:22 gerkey Exp $
  **************************************************************************/
 
 #ifndef PLAYERC_H
 #define PLAYERC_H
 
 #include <stdio.h>
 
 // Get the message structures from Player
 #include "player.h"
 
 #ifndef MIN
   #define MIN(a,b) ((a < b) ? a : b)
 #endif
 #ifndef MAX
   #define MAX(a,b) ((a > b) ? a : b)
 #endif
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
   
 /***************************************************************************
  * Useful constants (re-defined here so SWIG can pick them up easily)
  **************************************************************************/
 
 #define PLAYERC_READ_MODE PLAYER_READ_MODE
 #define PLAYERC_WRITE_MODE PLAYER_WRITE_MODE
 #define PLAYERC_ALL_MODE PLAYER_ALL_MODE
 #define PLAYERC_CLOSE_MODE PLAYER_CLOSE_MODE
 #define PLAYERC_ERROR_MODE PLAYER_ERROR_MODE
   
 #define PLAYERC_DATAMODE_PUSH_ALL PLAYER_DATAMODE_PUSH_ALL 
 #define PLAYERC_DATAMODE_PULL_ALL PLAYER_DATAMODE_PULL_ALL 
 #define PLAYERC_DATAMODE_PUSH_NEW PLAYER_DATAMODE_PUSH_NEW 
 #define PLAYERC_DATAMODE_PULL_NEW PLAYER_DATAMODE_PULL_NEW 
 #define PLAYERC_DATAMODE_PUSH_ASYNC PLAYER_DATAMODE_PUSH_ASYNC
 
 
 /***************************************************************************
  * Array sizes
  **************************************************************************/
 
 #define PLAYERC_MAX_DEVICES             PLAYER_MAX_DEVICES
 #define PLAYERC_LASER_MAX_SAMPLES       PLAYER_LASER_MAX_SAMPLES
 #define PLAYERC_FIDUCIAL_MAX_SAMPLES    PLAYER_FIDUCIAL_MAX_SAMPLES
 #define PLAYERC_SONAR_MAX_SAMPLES       PLAYER_SONAR_MAX_SAMPLES
 #define PLAYERC_BUMPER_MAX_SAMPLES      PLAYER_BUMPER_MAX_SAMPLES
 #define PLAYERC_IR_MAX_SAMPLES                PLAYER_IR_MAX_SAMPLES
 #define PLAYERC_BLOBFINDER_MAX_BLOBS    PLAYER_BLOBFINDER_MAX_BLOBS
 #define PLAYERC_WIFI_MAX_LINKS          PLAYER_WIFI_MAX_LINKS
 
   
 
 /***************************************************************************/
 /***************************************************************************/
 
 const char *playerc_error_str(void);
 
 const char *playerc_lookup_name(int code);
 
 int playerc_lookup_code(const char *name);
 
 /***************************************************************************/
 
   
 // Forward declare types
 struct _playerc_client_t;
 struct _playerc_device_t;
 
 
 // forward declaration to avoid including <sys/poll.h>, which may not be
 // available when people are building clients against this lib
 struct pollfd;
 
 
 /***************************************************************************/
 // Items in incoming data queue.
 typedef struct
 {
   player_msghdr_t header;
   int len;
   void *data;
 } playerc_client_item_t;
 
   
 // Multi-client data
 typedef struct
 {
   // List of clients being managed
   int client_count;
   struct _playerc_client_t *client[128];
 
   // Poll info 
   struct pollfd* pollfd;
 
   // Latest time received from any server
   double time;
 
 } playerc_mclient_t;
 
 // Create a multi-client object
 playerc_mclient_t *playerc_mclient_create(void);
 
 // Destroy a multi-client object
 void playerc_mclient_destroy(playerc_mclient_t *mclient);
 
 // Add a client to the multi-client (private).
 int playerc_mclient_addclient(playerc_mclient_t *mclient, struct _playerc_client_t *client);
 
 // Test to see if there is pending data.
 // Returns -1 on error, 0 or 1 otherwise.
 int playerc_mclient_peek(playerc_mclient_t *mclient, int timeout);
 
 // Read incoming data.  The timeout is in ms.  Set timeout to a
 // negative value to wait indefinitely.
 int playerc_mclient_read(playerc_mclient_t *mclient, int timeout);
 
 /***************************************************************************/
 
 
 /***************************************************************************/
 typedef void (*playerc_putdata_fn_t) (void *device, char *header, char *data, size_t len);
 
 typedef void (*playerc_callback_fn_t) (void *data);
 
 
 typedef struct
 {  
   int port, code, index;
 
   char drivername[PLAYER_MAX_DEVICE_STRING_LEN];
   
 } playerc_device_info_t;
 
 
 typedef struct _playerc_client_t
 {
   void *id;
 
   char *host;
   int port;
     
   int sock;
 
   int mode;
 
   playerc_device_info_t devinfos[PLAYERC_MAX_DEVICES];
   int devinfo_count;
 
   struct _playerc_device_t *device[32];
   int device_count;
 
   playerc_client_item_t qitems[128];
   int qfirst, qlen, qsize;
 
   char *data;
 
   double datatime;
 
 } playerc_client_t;
 
 
 playerc_client_t *playerc_client_create(playerc_mclient_t *mclient,
                                         const char *host, int port);
 
 void playerc_client_destroy(playerc_client_t *client);
 
 int playerc_client_connect(playerc_client_t *client);
 
 int playerc_client_disconnect(playerc_client_t *client);
 
 int playerc_client_datamode(playerc_client_t *client, int mode);
 
 int playerc_client_requestdata(playerc_client_t* client);
 
 int playerc_client_datafreq(playerc_client_t *client, int freq);
 
 int playerc_client_adddevice(playerc_client_t *client, struct _playerc_device_t *device);
 
 
 int playerc_client_deldevice(playerc_client_t *client, struct _playerc_device_t *device);
 
 int  playerc_client_addcallback(playerc_client_t *client, struct _playerc_device_t *device,
                                 playerc_callback_fn_t callback, void *data);
 
 int  playerc_client_delcallback(playerc_client_t *client, struct _playerc_device_t *device,
                                 playerc_callback_fn_t callback, void *data);
 
 int playerc_client_get_devlist(playerc_client_t *client);
 
 int playerc_client_subscribe(playerc_client_t *client, int code, int index,
                              int access, char *drivername, size_t len);
 
 int playerc_client_unsubscribe(playerc_client_t *client, int code, int index);
 
 int playerc_client_request(playerc_client_t *client, struct _playerc_device_t *device,
                            void *req_data, int req_len, void *rep_data, int rep_len);
                                 
 int playerc_client_peek(playerc_client_t *client, int timeout);
 
 void *playerc_client_read(playerc_client_t *client);
 
 int playerc_client_write(playerc_client_t *client, struct _playerc_device_t *device,
                          void *cmd, int len);
 
 
 /**************************************************************************/
 
 
 /***************************************************************************/
 typedef struct _playerc_device_t
 {
   void *id;
 
   playerc_client_t *client;
 
   int code, index;
 
   char drivername[PLAYER_MAX_DEVICE_STRING_LEN];
   
   int subscribed;
 
   double datatime;
 
   int fresh;
 
   playerc_putdata_fn_t putdata;
 
   void *user_data;
   
   int callback_count;
   playerc_callback_fn_t callback[4];
   void *callback_data[4];
 
 } playerc_device_t;
 
 
 void playerc_device_init(playerc_device_t *device, playerc_client_t *client,
                          int code, int index, playerc_putdata_fn_t putdata);
 
 void playerc_device_term(playerc_device_t *device);
 
 int playerc_device_subscribe(playerc_device_t *device, int access);
 
 int playerc_device_unsubscribe(playerc_device_t *device);
 
 /**************************************************************************/
 
 
 /***************************************************************************/
 /***************************************************************************/
 
 
 /***************************************************************************/
 typedef struct
 {  
   int id;
 
   uint32_t color;
 
   int x, y;
 
   int left, top, right, bottom;
 
   int area;
 
   double range;
   
 } playerc_blobfinder_blob_t;
 
 
 typedef struct 
 {
   playerc_device_t info;
 
   int width, height;
   
   int blob_count;
   playerc_blobfinder_blob_t blobs[PLAYERC_BLOBFINDER_MAX_BLOBS];
   
 } playerc_blobfinder_t;
 
 
 playerc_blobfinder_t *playerc_blobfinder_create(playerc_client_t *client, int index);
 
 void playerc_blobfinder_destroy(playerc_blobfinder_t *device);
 
 int playerc_blobfinder_subscribe(playerc_blobfinder_t *device, int access);
 
 int playerc_blobfinder_unsubscribe(playerc_blobfinder_t *device);
 
 void playerc_blobfinder_putdata(playerc_blobfinder_t *device, player_msghdr_t *header,
                                 player_blobfinder_data_t *data, size_t len);
 
 
 /**************************************************************************/
 
 
 /**************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 
   int pose_count;
   
   double poses[PLAYERC_BUMPER_MAX_SAMPLES][5];
   
   int bumper_count;
 
   double bumpers[PLAYERC_BUMPER_MAX_SAMPLES];
   
 } playerc_bumper_t;
 
 
 playerc_bumper_t *playerc_bumper_create(playerc_client_t *client, int index);
 
 void playerc_bumper_destroy(playerc_bumper_t *device);
 
 int playerc_bumper_subscribe(playerc_bumper_t *device, int access);
 
 int playerc_bumper_unsubscribe(playerc_bumper_t *device);
 
 int playerc_bumper_get_geom(playerc_bumper_t *device);
 
 
 /***************************************************************************/
 
 
 /***************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 
   int width, height;
 
   int bpp;
 
   int format;
 
   int fdiv;
 
   int compression;
 
   int image_size;
   
   uint8_t image[PLAYER_CAMERA_IMAGE_SIZE];
     
 } playerc_camera_t;
 
 
 playerc_camera_t *playerc_camera_create(playerc_client_t *client, int index);
 
 void playerc_camera_destroy(playerc_camera_t *device);
 
 int playerc_camera_subscribe(playerc_camera_t *device, int access);
 
 int playerc_camera_unsubscribe(playerc_camera_t *device);
 
 void playerc_camera_decompress(playerc_camera_t *device);
 
 
 /**************************************************************************/
 
 /***************************************************************************/
 typedef struct
 {
   int id;
 
   double range, bearing, orient;
 
   double pos[3];
 
   double rot[3];
 
   double upos[3];
 
   double urot[3];
 
 } playerc_fiducial_item_t;
 
 
 typedef struct
 {
   playerc_device_t info;
 
   double pose[3];
   double size[2];
   double fiducial_size[2];
   
   int fiducial_count;
   playerc_fiducial_item_t fiducials[PLAYERC_FIDUCIAL_MAX_SAMPLES];
     
 } playerc_fiducial_t;
 
 
 playerc_fiducial_t *playerc_fiducial_create(playerc_client_t *client, int index);
 
 void playerc_fiducial_destroy(playerc_fiducial_t *device);
 
 int playerc_fiducial_subscribe(playerc_fiducial_t *device, int access);
 
 int playerc_fiducial_unsubscribe(playerc_fiducial_t *device);
 
 int playerc_fiducial_get_geom(playerc_fiducial_t *device);
 
 
 /**************************************************************************/
 
 
 /***************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 
   double utc_time;
   
   double lat, lon;
 
   double alt;
 
   double utm_e, utm_n;
 
   double hdop;
 
   double vdop;
 
   double err_horz, err_vert;
 
   int quality;
      
   int sat_count;
 
 } playerc_gps_t;
 
 
 playerc_gps_t *playerc_gps_create(playerc_client_t *client, int index);
 
 void playerc_gps_destroy(playerc_gps_t *device);
 
 int playerc_gps_subscribe(playerc_gps_t *device, int access);
 
 int playerc_gps_unsubscribe(playerc_gps_t *device);
 
 
 /**************************************************************************/
 
 /***************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 
   // data
   player_ir_data_t ranges;
   
   // config
   player_ir_pose_t poses;
 
 } playerc_ir_t;
 
 
 playerc_ir_t *playerc_ir_create(playerc_client_t *client, int index);
 
 void playerc_ir_destroy(playerc_ir_t *device);
 
 int playerc_ir_subscribe(playerc_ir_t *device, int access);
 
 int playerc_ir_unsubscribe(playerc_ir_t *device);
 
 int playerc_ir_get_geom(playerc_ir_t *device);
 
 
 /***************************************************************************/
 
 
 /***************************************************************************/
 typedef struct
 {
   playerc_device_t info;
   
   double px, py;
 
   uint16_t buttons;
   
 } playerc_joystick_t;
 
 
 playerc_joystick_t *playerc_joystick_create(playerc_client_t *client, int index);
 
 void playerc_joystick_destroy(playerc_joystick_t *device);
 
 int playerc_joystick_subscribe(playerc_joystick_t *device, int access);
 
 int playerc_joystick_unsubscribe(playerc_joystick_t *device);
 
 void playerc_joystick_putdata(playerc_joystick_t *device, player_msghdr_t *header,
                               player_joystick_data_t *data, size_t len);
 
 
 /**************************************************************************/
 
 
 /***************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 
   double pose[3];
   double size[2];
   
   int scan_count;
 
   double scan_start;
 
   double scan_res;
 
   int range_res;
 
   double ranges[PLAYERC_LASER_MAX_SAMPLES];
 
   double scan[PLAYERC_LASER_MAX_SAMPLES][2];
   
   double point[PLAYERC_LASER_MAX_SAMPLES][2];
 
   int intensity[PLAYERC_LASER_MAX_SAMPLES];
   
 } playerc_laser_t;
 
 
 playerc_laser_t *playerc_laser_create(playerc_client_t *client, int index);
 
 void playerc_laser_destroy(playerc_laser_t *device);
 
 int playerc_laser_subscribe(playerc_laser_t *device, int access);
 
 int playerc_laser_unsubscribe(playerc_laser_t *device);
 
 void playerc_laser_putdata(playerc_laser_t *device, player_msghdr_t *header,
                            player_laser_data_t *data, size_t len);
 
 int playerc_laser_set_config(playerc_laser_t *device,
                              double min_angle, double max_angle,
                              int resolution, int range_res, int intensity);
 
 int playerc_laser_get_config(playerc_laser_t *device,
                              double *min_angle, double *max_angle,
                              int *resolution, int *range_res, int *intensity);
 
 int playerc_laser_get_geom(playerc_laser_t *device);
 
 /**************************************************************************/
 
 /***************************************************************************/
 typedef struct
 {
   double mean[3];
 
   double cov[3][3];
 
   double weight;
   
 } playerc_localize_hypoth_t;
 
 typedef struct playerc_localize_particle
 {
   double pose[3];
   double weight;
 } playerc_localize_particle_t;
 
 
 typedef struct
 {
   playerc_device_t info;
 
   int map_size_x, map_size_y;
   
   double map_scale;
 
   int map_tile_x, map_tile_y;
   
   int8_t *map_cells;
 
   int pending_count;
 
   double pending_time;
 
   int hypoth_count;
   playerc_localize_hypoth_t hypoths[PLAYER_LOCALIZE_MAX_HYPOTHS];
 
   double mean[3];
   double variance;
   int num_particles;
   playerc_localize_particle_t particles[PLAYER_LOCALIZE_PARTICLES_MAX];
 
 } playerc_localize_t;
 
 
 playerc_localize_t *playerc_localize_create(playerc_client_t *client, int index);
 
 void playerc_localize_destroy(playerc_localize_t *device);
 
 int playerc_localize_subscribe(playerc_localize_t *device, int access);
 
 int playerc_localize_unsubscribe(playerc_localize_t *device);
 
 int playerc_localize_set_pose(playerc_localize_t *device, double pose[3], double cov[3][3]);
 
 int playerc_localize_get_config(playerc_localize_t *device, player_localize_config_t *config) int playerc_localize_get_config(playerc_localize_t *device, player_localize_config_t *config) 
 int playerc_localize_set_config(playerc_localize_t *device, player_localize_config_t config); int playerc_localize_set_config(playerc_localize_t *device, player_localize_config_t config); 
 /* @brief Get the particle set.  Caller must supply sufficient storage for
    the result. */
 int playerc_localize_get_particles(playerc_localize_t *device);
 
 /**************************************************************************/
 
 
 /***************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 
   int type;
 
   int state;
 } playerc_log_t;
 
 
 playerc_log_t *playerc_log_create(playerc_client_t *client, int index);
 
 void playerc_log_destroy(playerc_log_t *device);
 
 int playerc_log_subscribe(playerc_log_t *device, int access);
 
 int playerc_log_unsubscribe(playerc_log_t *device);
 
 int playerc_log_set_write_state(playerc_log_t* device, int state);
 
 int playerc_log_set_read_state(playerc_log_t* device, int state);
 
 int playerc_log_set_read_rewind(playerc_log_t* device);
 
 int playerc_log_get_state(playerc_log_t* device);
 
 int playerc_log_set_filename(playerc_log_t* device, const char* fname);
 
 
 /***************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 
   double resolution;
 
   int width, height;
 
   char* cells;
 } playerc_map_t;
 
 
 #define PLAYERC_MAP_INDEX(dev, i, j) ((dev->width) * (j) + (i))
 
 playerc_map_t *playerc_map_create(playerc_client_t *client, int index);
 
 void playerc_map_destroy(playerc_map_t *device);
 
 int playerc_map_subscribe(playerc_map_t *device, int access);
 
 int playerc_map_unsubscribe(playerc_map_t *device);
 
 int playerc_map_get_map(playerc_map_t* device);
 
 
 /**************************************************************************/
 
 
 /***************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 
   double pt;
 
   double vt;
 
   int stall;
 
 } playerc_motor_t;
 
 
 playerc_motor_t *playerc_motor_create(playerc_client_t *client, int index);
 
 void playerc_motor_destroy(playerc_motor_t *device);
 
 int playerc_motor_subscribe(playerc_motor_t *device, int access);
 
 int playerc_motor_unsubscribe(playerc_motor_t *device);
 
 int playerc_motor_enable(playerc_motor_t *device, int enable);
 
 int playerc_motor_position_control(playerc_motor_t *device, int type);
 
 int playerc_motor_set_cmd_vel(playerc_motor_t *device,
                               double vt, int state);
 
 int playerc_motor_set_cmd_pose(playerc_motor_t *device,
                                double gt, int state);
 
 /**************************************************************************/
 
 /***************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 
   int path_valid;
 
   int path_done;
 
   double px, py, pa;
 
   double gx, gy, ga;
 
   double wx, wy, wa;
 
   int curr_waypoint;
 
   int waypoint_count;
 
   double waypoints[PLAYER_PLANNER_MAX_WAYPOINTS][3];
   
 } playerc_planner_t;
 
 playerc_planner_t *playerc_planner_create(playerc_client_t *client, int index);
 
 void playerc_planner_destroy(playerc_planner_t *device);
 
 int playerc_planner_subscribe(playerc_planner_t *device, int access);
 
 int playerc_planner_unsubscribe(playerc_planner_t *device);
 
 int playerc_planner_set_cmd_pose(playerc_planner_t *device,
                                   double gx, double gy, double ga);
 
 int playerc_planner_get_waypoints(playerc_planner_t *device);
 
 int playerc_planner_enable(playerc_planner_t *device, int state);
 
 /**************************************************************************/
 
 
 /***************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 
   double pose[3];
   double size[2];
   
   double px, py, pa;
 
   double vx, vy, va;
   
   int stall;
 } playerc_position_t;
 
 
 playerc_position_t *playerc_position_create(playerc_client_t *client, int index);
 
 void playerc_position_destroy(playerc_position_t *device);
 
 int playerc_position_subscribe(playerc_position_t *device, int access);
 
 int playerc_position_unsubscribe(playerc_position_t *device);
 
 void playerc_position_putdata(playerc_position_t *device, player_msghdr_t *header,
                               player_position_data_t *data, size_t len);
 
 
 int playerc_position_enable(playerc_position_t *device, int enable);
 
 int playerc_position_get_geom(playerc_position_t *device);
 
 int playerc_position_set_cmd_vel(playerc_position_t *device,
                                  double vx, double vy, double va, int state);
 
 int playerc_position_set_cmd_pose(playerc_position_t *device,
                                   double gx, double gy, double ga, int state);
 
 
 /**************************************************************************/
 
  
 /***************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 
   double pose[3];
   double size[2];
 
   double px, py, pa;
 
   double vx, vy, va;
 
   int stall;
 
 } playerc_position2d_t;
 
 
 playerc_position2d_t *playerc_position2d_create(playerc_client_t *client, int index);
 
 void playerc_position2d_destroy(playerc_position2d_t *device);
 
 int playerc_position2d_subscribe(playerc_position2d_t *device, int access);
 
 int playerc_position2d_unsubscribe(playerc_position2d_t *device);
 
 int playerc_position2d_enable(playerc_position2d_t *device, int enable);
 
 int playerc_position2d_get_geom(playerc_position2d_t *device);
 
 int playerc_position2d_set_cmd_vel(playerc_position2d_t *device,
                                    double vx, double vy, double va, int state);
 
 int playerc_position2d_set_cmd_pose(playerc_position2d_t *device,
                                     double gx, double gy, double ga, int state);
 
 /**************************************************************************/
 
 
 /***************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 
   double pose[3];
   double size[2];
 
   double pos_x, pos_y, pos_z;
 
   double pos_roll, pos_pitch, pos_yaw;
 
   double vel_x, vel_y, vel_z;
 
   double vel_roll, vel_pitch, vel_yaw;
 
   int stall;
 
 } playerc_position3d_t;
 
 
 playerc_position3d_t *playerc_position3d_create(playerc_client_t *client,
                                                 int index);
 
 void playerc_position3d_destroy(playerc_position3d_t *device);
 
 int playerc_position3d_subscribe(playerc_position3d_t *device, int access);
 
 int playerc_position3d_unsubscribe(playerc_position3d_t *device);
 
 void playerc_position3d_putdata(playerc_position3d_t *device, player_msghdr_t *header,
                                 player_position3d_data_t *data, size_t len);
 
 int playerc_position3d_enable(playerc_position3d_t *device, int enable);
 
 int playerc_position3d_get_geom(playerc_position3d_t *device);
 
 int playerc_position3d_set_velocity(playerc_position3d_t *device,
                                     double vx, double vy, double vz,
                                     double vr, double vp, double vt, int state);
 
 int playerc_position3d_set_speed(playerc_position3d_t *device,
                                  double vx, double vy, double vz, int state);
 
 int playerc_position3d_set_pose(playerc_position3d_t *device,
                                 double gx, double gy, double gz,
                                 double gr, double gp, double gt);
 
 int playerc_position3d_set_cmd_pose(playerc_position3d_t *device,
                                     double gx, double gy, double gz);
 
 /**************************************************************************/
 
 /***************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 
   double charge;
   
 } playerc_power_t;
 
 
 playerc_power_t *playerc_power_create(playerc_client_t *client, int index);
 
 void playerc_power_destroy(playerc_power_t *device);
 
 int playerc_power_subscribe(playerc_power_t *device, int access);
 
 int playerc_power_unsubscribe(playerc_power_t *device);
 
 
 /**************************************************************************/ 
 
 
 /***************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 
   double pan, tilt;
 
   double zoom;
   
 } playerc_ptz_t;
 
 
 playerc_ptz_t *playerc_ptz_create(playerc_client_t *client, int index);
 
 void playerc_ptz_destroy(playerc_ptz_t *device);
 
 int playerc_ptz_subscribe(playerc_ptz_t *device, int access);
 
 int playerc_ptz_unsubscribe(playerc_ptz_t *device);
 
 int playerc_ptz_set(playerc_ptz_t *device, double pan, double tilt, double zoom);
 
 int playerc_ptz_set_ws(playerc_ptz_t *device, double pan, double tilt, double zoom,
                        double panspeed, double tiltspeed);
 
 
 /**************************************************************************/ 
 
 
 /***************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 
   int pose_count;
   
   double poses[PLAYERC_SONAR_MAX_SAMPLES][3];
   
   int scan_count;
 
   double scan[PLAYERC_SONAR_MAX_SAMPLES];
   
 } playerc_sonar_t;
 
 
 playerc_sonar_t *playerc_sonar_create(playerc_client_t *client, int index);
 
 void playerc_sonar_destroy(playerc_sonar_t *device);
 
 int playerc_sonar_subscribe(playerc_sonar_t *device, int access);
 
 int playerc_sonar_unsubscribe(playerc_sonar_t *device);
 
 int playerc_sonar_get_geom(playerc_sonar_t *device);
 
 /**************************************************************************/
 
 
 /***************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 
   double pos[3];
 
   double rot[3];
 
 } playerc_truth_t;
 
 
 playerc_truth_t *playerc_truth_create(playerc_client_t *client, int index);
 
 void playerc_truth_destroy(playerc_truth_t *device);
 
 int playerc_truth_subscribe(playerc_truth_t *device, int access);
 
 int playerc_truth_unsubscribe(playerc_truth_t *device);
 
 int playerc_truth_get_pose(playerc_truth_t *device,
                            double *px, double *py, double *pz,
                            double *rx, double *ry, double *rz);
 
 int playerc_truth_set_pose(playerc_truth_t *device,
                            double px, double py, double pz,
                            double rx, double ry, double rz);
 
 
 /***************************************************************************/
 
 
 /***************************************************************************/
 typedef struct
 {
   char mac[32];
 
   char ip[32];
 
   char essid[32];
 
   int mode;
 
   int encrypt;
 
   double freq;
 
   int qual, level, noise;
  
 } playerc_wifi_link_t;
 
 
 typedef struct
 {
   playerc_device_t info;
 
   playerc_wifi_link_t links[PLAYERC_WIFI_MAX_LINKS];
   int link_count;
   
 } playerc_wifi_t;
 
 
 playerc_wifi_t *playerc_wifi_create(playerc_client_t *client, int index);
 
 void playerc_wifi_destroy(playerc_wifi_t *device);
 
 int playerc_wifi_subscribe(playerc_wifi_t *device, int access);
 
 int playerc_wifi_unsubscribe(playerc_wifi_t *device);
 
 playerc_wifi_link_t *playerc_wifi_get_link(playerc_wifi_t *device, int link);
 
 typedef struct
 {
   playerc_device_t info;
 
 } playerc_simulation_t;
 
 
 // Create a new simulation proxy
 playerc_simulation_t *playerc_simulation_create(playerc_client_t *client, int index);
 
 // Destroy a simulation proxy
 void playerc_simulation_destroy(playerc_simulation_t *device);
 
 // Subscribe to the simulation device
 int playerc_simulation_subscribe(playerc_simulation_t *device, int access);
 
 // Un-subscribe from the simulation device
 int playerc_simulation_unsubscribe(playerc_simulation_t *device);
 
 // Process incoming data
 void playerc_simulation_putdata(playerc_simulation_t *device, player_msghdr_t *header,
                                 player_simulation_data_t *data, size_t len);
 
 int playerc_simulation_set_pose2d(playerc_simulation_t *device, char* name,
                                   double gx, double gy, double ga);
 
 int playerc_simulation_get_pose2d(playerc_simulation_t *device, char* identifier, 
                                   double *x, double *y, double *a);
 
 /***************************************************************************/
 
 /***************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 
   unsigned char state;
   unsigned char beams;
   int outer_break_beam;
   int inner_break_beam;
   int paddles_open;
   int paddles_closed;
   int paddles_moving;
   int gripper_error;
   int lift_up;
   int lift_down;
   int lift_moving;
   int lift_error;
 
 } playerc_gripper_t;
 
 
 playerc_gripper_t *playerc_gripper_create(playerc_client_t *client, int index);
 
 void playerc_gripper_destroy(playerc_gripper_t *device);
 
 int playerc_gripper_subscribe(playerc_gripper_t *device, int access);
 
 int playerc_gripper_unsubscribe(playerc_gripper_t *device);
 
 int playerc_gripper_set_cmd(playerc_gripper_t *device, uint8_t cmd, uint8_t arg);
 
 /**************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 
     uint8_t count;
 
     uint32_t digin;
   
 } playerc_dio_t;
 
 
 playerc_dio_t *playerc_dio_create(playerc_client_t *client, int index);
 
 void playerc_dio_destroy(playerc_dio_t *device);
 
 int playerc_dio_subscribe(playerc_dio_t *device, int access);
 
 int playerc_dio_unsubscribe(playerc_dio_t *device);
 
 int playerc_dio_set_output(playerc_dio_t *device, uint8_t output_count, uint32_t digout);
 
 
 /***************************************************************************/
 
 
 /**************************************************************************/
 typedef struct
 {
   playerc_device_t info;
 } playerc_speech_t;
 
 
 playerc_speech_t *playerc_speech_create(playerc_client_t *client, int index);
 
 void playerc_speech_destroy(playerc_speech_t *device);
 
 int playerc_speech_subscribe(playerc_speech_t *device, int access);
 
 int playerc_speech_unsubscribe(playerc_speech_t *device);
 
 int playerc_speech_say (playerc_speech_t *device, char *);
 
 
 /***************************************************************************/
 
 
 /**************************************************************************/
 /**************************************************************************/
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif

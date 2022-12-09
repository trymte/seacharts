from __future__ import annotations

from rclpy.node import Node
import datetime
import time
import tkinter as tk
from multiprocessing import Process
from typing import List, Tuple

import numpy as np
import matplotlib.pyplot as plt
import simcharts.environment as env
from cartopy.crs import UTM
from matplotlib.gridspec import GridSpec

from simcharts.utils.helper import *

from .colors import colorbar
from .events import EventsManager
from .features import FeaturesManager


class Display:
    crs = None
    window_anchors = (
        ('top_left', 'top', 'top_right'),
        ('left', 'center', 'right'),
        ('bottom_left', 'bottom', 'bottom_right'),
    )

    def __init__(self, settings: dict, environment: env.Environment = None, node: Node = None):
        if environment is None:
            self.environment = env.Environment()
        else:
            self.environment = environment
        self.node = node
        self.crs = UTM(settings['enc']['utm_zone'])
        self.draw_names = settings['display']['draw_names']
        self.nr_of_shadow_ships = settings['display']['nr_of_shadow_ships']

        self._background = None
        self.anchor_index = self._init_anchor_index(settings)
        self.figure, self.sizes, self.spacing, widths = self._init_figure(settings)
        self.axes, self.grid_spec, self._colorbar = self._init_axes(widths)
        self.events = EventsManager(self)
        self.features = FeaturesManager(self)
        self.draw_plot()

        if self._fullscreen_mode:
            self.toggle_fullscreen()
        else:
            self.set_figure_position()

        if self._colorbar_mode:
            self.toggle_colorbar()

        if self._dark_mode:
            self.toggle_dark_mode()
        if environment is None:
            self.start_visualization_loop()

    def _init_anchor_index(self, settings):
        option = settings['display']['anchor']
        for j, window_anchor in enumerate(self.window_anchors):
            if option in window_anchor:
                return j, window_anchor.index(option)
        raise ValueError(
            f"Invalid window anchor option '{option}', "
            f"possible candidates are: \n"
            f"{[o for options in self.window_anchors for o in options]}"
        )

    def _init_figure(self, settings):
        self._fullscreen_mode = settings['display']['fullscreen_mode']
        self._colorbar_mode = settings['display']['colorbar_mode']
        self._dark_mode = settings['display']['dark_mode']
        self._dpi = settings['display']['dpi']
        self._resolution = settings['display']['resolution']

        if self._fullscreen_mode:
            plt.rcParams['toolbar'] = 'None'

        width, height = self.environment.scope.extent.size
        window_height, ratio = self._resolution / self._dpi, width / height
        figure_width1, figure_height1 = ratio * window_height, window_height
        axes1_width, axes2_width, width_space = figure_width1, 1.1, 0.3
        axes_widths = axes1_width, axes2_width
        figure_height2 = figure_height1 * 0.998
        figure_width2 = axes1_width + width_space + 2 * axes2_width
        figure_sizes = [(figure_width1, figure_height1),
                        (figure_width2, figure_height2)]
        sub1 = dict(
            right=(axes1_width + width_space + axes2_width) / figure_width1,
            wspace=2 * width_space / (axes1_width + axes2_width),
        )
        sub2 = dict(
            right=(axes1_width + width_space + axes2_width) / figure_width2,
            wspace=2 * width_space / axes1_width,
        )
        subplot_spacing = sub1, sub2
        # enable interactive mode
        plt.ion()
        figure = plt.figure('SeaCharts', figsize=figure_sizes[0], dpi=self._dpi)
        if not self._fullscreen_mode:
            figure.canvas.toolbar.pack_forget()
        return figure, figure_sizes, subplot_spacing, axes_widths

    def _init_axes(self, axes_widths):
        gs = GridSpec(1, 2, width_ratios=axes_widths, **self.spacing[0],
                      left=0.0, top=1.0, bottom=0.0, hspace=0.0)
        axes1 = self.figure.add_subplot(gs[0, 0], projection=self.crs)
        x_min, y_min, x_max, y_max = self.environment.scope.extent.bbox
        axes1.set_extent((x_min, x_max, y_min, y_max), crs=self.crs)
        # axes1.background_patch.set_visible(False)
        # axes1.outline_patch.set_visible(False)
        axes2 = self.figure.add_subplot(gs[0, 1])
        cb = colorbar(axes2, self.environment.scope.depths)
        return axes1, gs, cb

    def start_visualization_loop(self):
        self.show(0.1)
        start_time = time.time()
        while True:
            delta = datetime.timedelta(seconds=time.time() - start_time)
            now = str(delta).split(".")[0]
            print(
                f"\rVisualizing multiprocessing environment | {now}", end=''
            )
            if not self.is_active:
                self.terminate()
                print()
                return
            self.features.update_vessels_from_file()
            self.update_plot()
            time.sleep(0.1)

    def refresh_vessels(self, vessels, size, origin):
        '''
        Refreshes the vessels in the environment

        In: 
            vessels: (Vessel[]) List of Vessel messages
        '''
        if vessels != {}: self.features.update_vessels(vessels, size, origin)

    def refresh_vessels_from_file(self, poses: List[Tuple]):
        self.features.vessels_to_file(poses)
        self.features.update_vessels_from_file()
        self.update_plot()

    def update_plot(self):
        self.figure.canvas.restore_region(self._background)
        self.node.get_logger().debug("Updating entire plot")
        self.draw_animated_artists()

    def clean_plot(self):
        self.figure.canvas.restore_region(self._background)
        self.node.get_logger().debug("Cleaning entire plot")
        self.remove_animated_paths()
        self.draw_animated_artists()

    def update_static_plot(self):
        self.figure.canvas.restore_region(self._background)
        self.node.get_logger().debug("Updating static plot")
        self.draw_animated_static()

    def update_vessels_plot(self):
        self.figure.canvas.restore_region(self._background)
        self.draw_animated_vessels()
        if self.draw_names:
            self.draw_animated_vessels_text()

    def draw_plot(self):
        try:
            self.figure.canvas.draw()
        except tk.TclError:
            plt.close()
        self._background = self.figure.canvas.copy_from_bbox(self.figure.bbox)
        self.draw_animated_artists()

    def draw_animated_artists(self):
        for artist in self.features.animated:
            self.axes.draw_artist(artist)
        try:
            self.figure.canvas.blit()
            self.figure.canvas.flush_events()
        except tk.TclError:
            plt.close()

    def draw_animated_static(self):
        for artist in self.features.animatedStatic:
            self.axes.draw_artist(artist)
        try:
            self.figure.canvas.blit()
            self.figure.canvas.flush_events()
        except tk.TclError:
            plt.close()

    def draw_animated_vessels(self):
        for artist in self.features.animatedVessels:
            self.axes.draw_artist(artist)
        try:
            self.figure.canvas.blit()
            self.figure.canvas.flush_events()
        except tk.TclError:
            plt.close()

    def draw_animated_shadows(self):
        for artist in self.features.animatedShadowShips:
            self.axes.draw_artist(artist)
        try:
            self.figure.canvas.blit()
            self.figure.canvas.flush_events()
        except tk.TclError:
            plt.close()

    def remove_animated_paths(self):
        for artist in self.features.animatedPaths:
            artist.remove()
        try:
            self.figure.canvas.blit()
            self.figure.canvas.flush_events()
        except tk.TclError:
            plt.close()

    def draw_animated_vessels_text(self):
        for artist in self.features.animatedVesselsText:
            self.axes.draw_artist(artist)
        try:
            self.figure.canvas.blit()
            self.figure.canvas.flush_events()
        except tk.TclError:
            plt.close()

    def draw_path(self, queue):
        for id, path_obj in queue.items():
            # If a path with the same id already exists, extend it
            p = path_obj['path']
            color = path_obj['color']
            if id in self.features.inputted_paths:
                p = np.vstack([self.features.inputted_paths[id]['path'], p])
                color = self.features.inputted_paths[id]['color']

            artist = self.features.add_line(p[:,:2], color, path_obj['buffer'], path_obj['thickness'], path_obj['edge_style'])
            self.features.inputted_paths[id] = {}
            self.features.inputted_paths[id]['artist'] = artist
            self.features.inputted_paths[id]['path'] = p
            self.features.inputted_paths[id]['color'] = color

            # Update the local traffic object, if there are any associated with the trajectory
            # if id in self.features._vessels:
            self.node.get_logger().debug(f"Ship with id {id} has a path, updating local traffic object")
            self.features.draw_shadow_ships(id, p, self.nr_of_shadow_ships)

    def draw_animated_trajectory(self, queue):
        t_now = float(getTimeStamp())
        pop_ids = []
        self.figure.canvas.restore_region(self._background)
        for id, trajectory_obj in queue.items():
            # If a trajectory with the same id already exists, extend it
            if id not in self.features.inputted_trajectories:
                self.features.inputted_trajectories[id] = {}
                self.features.inputted_trajectories[id]['t_start'] = t_now
                self.features.inputted_trajectories[id]['artist'] = []
                self.features.inputted_trajectories[id]['trajectory'] = []
                self.features.inputted_trajectories[id]['color'] = trajectory_obj['color']

            t_start = self.features.inputted_trajectories[id]['t_start']
            color = self.features.inputted_trajectories[id]['color']
            traversed_traj = self.features.inputted_trajectories[id]['trajectory']
            
            full_traj = trajectory_obj['trajectory']
            t = trajectory_obj['time']
            len_full_traj = len(trajectory_obj['trajectory'])
            len_trav_traj = len(traversed_traj)
            
            if t[len_trav_traj] + t_start > t_now:
                continue

            for i in range(len_trav_traj, len_full_traj):
                if t_now > t[i] + t_start:
                    traversed_traj.append(full_traj[i])
                else:
                    break
            len_trav_traj = len(traversed_traj)

            self.features.inputted_trajectories[id]['trajectory'] = traversed_traj
            
            if len_trav_traj > 1:
                artist = self.features.add_line(traversed_traj, color, trajectory_obj['buffer'], trajectory_obj['thickness'], trajectory_obj['edge_style'])
                artist.set_animated(True)
                self.features.inputted_trajectories[id]['artist'] = artist

                # Update the local traffic object, if there are any associated with the trajectory
                if id in self.features._vessels:
                    # self.features._vessels[id]['artist'].remove()
                    # self.features._vessels.pop(id)
                    self.node.local_traffic[id].x = traversed_traj[-1][0]
                    self.node.local_traffic[id].y = traversed_traj[-1][1]
                    self.node.local_traffic[id].heading = traversed_traj[-1][2]
                    self.features.update_vessel(self.node.local_traffic[id])

            if len_trav_traj == len_full_traj:
                pop_ids.append(id)
            return pop_ids

    # def draw_init_traj_pose(self, pose):


    def toggle_dark_mode(self, state=None):
        state = state if state is not None else not self._dark_mode
        color = '#142c38' if state else '#ffffff'
        self.figure.set_facecolor(color)
        self._colorbar.ax.set_facecolor(color)
        self.features.toggle_topography_visibility(not state)
        self._dark_mode = state
        self.draw_plot()

    def toggle_colorbar(self, state=None):
        if state is not None:
            self._colorbar_mode = state
        else:
            self._colorbar_mode = not self._colorbar_mode
        self.grid_spec.update(**self.spacing[int(self._colorbar_mode)])
        if not self._fullscreen_mode:
            self.figure.set_size_inches(self.sizes[int(self._colorbar_mode)])
            self.set_figure_position()
        self.draw_plot()

    def toggle_fullscreen(self, state=None):
        if state is not None:
            self._fullscreen_mode = state
        else:
            self._fullscreen_mode = not self._fullscreen_mode
        plt.get_current_fig_manager().full_screen_toggle()
        if not self._fullscreen_mode:
            self.figure.set_size_inches(self.sizes[int(self._colorbar_mode)])
            self.set_figure_position()
        self.draw_plot()

    def set_figure_position(self):
        j, i = self.anchor_index
        option = self.window_anchors[j][i]
        if option != 'default':
            root = tk.Tk()
            screen_width = int(root.winfo_screenwidth())
            screen_height = int(root.winfo_screenheight())
            root.destroy()
            x_margin, y_margin = -10, -73
            dpi = self._dpi
            size = self.sizes[int(self._colorbar_mode)]
            width, height = [int(size[k] * dpi) for k in range(2)]
            x_shifted = screen_width - width
            y_shifted = screen_height - height
            if option == 'center':
                x, y = x_shifted // 2, y_shifted // 2
            elif option == 'right':
                x, y = x_shifted, y_shifted // 2
            elif option == 'left':
                x, y = 4, y_shifted // 2
            elif option == 'top':
                x, y = x_shifted // 2, 2
            elif option == 'bottom':
                x, y = x_shifted // 2, y_shifted + y_margin
            elif option == 'top_right':
                x, y = x_shifted, 2
            elif option == 'top_left':
                x, y = 4, 2
            elif option == 'bottom_right':
                x, y = x_shifted, y_shifted + y_margin
            elif option == 'bottom_left':
                x, y = 4, y_shifted + y_margin
            else:
                x, y = 4, 2
            manager = plt.get_current_fig_manager()
            manager.window.wm_geometry(f"+{x + x_margin}+{y}")

    def save_figure(self, name=None, scale=1.0, extension='png'):
        try:
            if name is None:
                name = self.figure.canvas.manager.get_window_title()
            self.figure.savefig(
                f"reports/{name}.{extension}", dpi=self.figure.dpi * scale,
                bbox_inches=self.figure.bbox_inches, pad_inches=0.0,
            )
        except tk.TclError:
            plt.close()

    @property
    def is_active(self):
        return plt.fignum_exists(self.figure.number)

    def show(self, duration=0.0):
        if self.environment.ownship:
            self.features.update_ownship()
            if self.environment.safe_area:
                self.features.update_hazards()
        # try:
        #     print(f"B0: {duration}")
        #     plt.pause(duration)
        #     print("B")
        # except tk.TclError:
        #     plt.close()
        # print("B1")
        

    @staticmethod
    def terminate():
        plt.close()

    @staticmethod
    def init_multiprocessing(display):
        print("Initializing multiprocessing environment")
        Process(target=display.start_visualization_loop).start()

#!/usr/bin/python3
import rosbag2_py
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sys
import math
import yaml
import utm
import glob
import importlib.util
import os


class CustomArrayContainer:
    def __init__(self):
        self.standalones = []
        self.standalones_backup = []
        self.members = []
        self.members_backup = []

    def add_standalone(self, name):
        self.standalones.append(name)

    def add_member(self, obj, field):
        self.members.append((obj, field))

    def slice(self, start_time, end_time, context):
        self.members_backup = []
        for obj, field in self.members:
            a = getattr(obj, field)
            self.members_backup.append(a)
            setattr(obj, field, a[np.where(np.logical_and(a.bag_times >= start_time, a.bag_times <= end_time))])

        self.standalones_backup = []
        for k in self.standalones:
            a = context[k]
            self.standalones_backup.append((k, a.copy()))
            context[k] = a[np.where(np.logical_and(a.bag_times >= start_time, a.bag_times <= end_time))]

    def unslice(self, context):
        for i, (obj, field) in enumerate(self.members):
            setattr(obj, field, self.members_backup[i])

        for k, a in self.standalones_backup:
            context[k] = a

custom_array_container = CustomArrayContainer()

def get_time(stamp):
    return stamp.sec + stamp.nanosec*1e-9

class CustomArray(np.ndarray):
    def __new__(cls, input_array):
        obj = np.asarray(input_array).view(cls)
        obj.bag_times = None
        return obj

    def __get_attribute__(self, name):
        print('name', name)
        return super().__getattribute__(name)

    def __array_finalize__(self, obj):
        if obj is None:
            return
        self.bag_times = getattr(obj, 'bag_times', None)

def make_list(o):
    if type(o) in [tuple, set]:
        return list(o)
    else:
        return [o]

def apply_function(variable, function_name, function_arguments, extra_context={}):
    if function_name not in dir(variable):
        print('warning: ' + function_name + ' not a member of ' + str(type(variable)) + '. skipping.')
        return
    try:
        context = {'variable': variable}
        context.update(extra_context)
        eval('variable.' + function_name + '(' + (function_arguments if function_arguments != None else '') + ')',
             globals(), context)
    except Exception as e:
        context = {'variable': variable}
        context.update(extra_context)
        eval('variable.' + function_name + '(\'' + (function_arguments if function_arguments != None else '') + '\')',
             globals(), context)

def apply_profiles(o, profiles, keys):
    combined_profile = {}
    combined_context = {}
    for k in keys:
        if type(k) == dict:
            key, context = k['key'], k['kwargs']
        if callable(k):
            key, context = k()['key'], k()['kwargs']
        profile = profiles[key].copy()
        combined_profile.update(profile)
        combined_context.update(context)
    combined_profile.update(o)
    o.update(combined_profile)
    #del o['profiles']
    return combined_context

def get_subplot_params(total_rows, total_cols, row, col):
    if row < 1 or col < 1 or row > total_rows or col > total_cols:
        raise ValueError("Row and column must be within the subplot grid (1-indexed).")
    index = (row - 1) * total_cols + col
    return total_rows, total_cols, index

def get_row_col_info(axes):
    contains_row = ['row' in ax for ax in axes]
    contains_col = ['col' in ax for ax in axes]

    if not (all(contains_row) or not any(contains_row)):
        print('ERROR: a list of axes need to either all contain a row field or have none contain a row field\nfor ' + str(axes))
        exit()
        
    if not (all(contains_col) or not any(contains_col)):
        print('ERROR: a list of axes need to either all contain a col field or have none contain a col field\nfor ' + str(axes))
        exit()

    for i, ax in enumerate(axes):
        if not any(contains_row) and not any(contains_col):
            ax['row'] = i+1
            ax['col'] = 1
        else:
            if not any(contains_row):
                ax['row'] = 1
            if not any(contains_col):
                ax['col'] = 1

    total_rows = max([ax['row'] for ax in axes])
    total_cols = max([ax['col'] for ax in axes])

    return total_rows, total_cols

def get_module(script_path):
    module_name = os.path.splitext(os.path.basename(script_path))[0]
    spec = importlib.util.spec_from_file_location(module_name, script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

def handle_scripts(dct):
    if 'script' not in dct:
        return dct

    module = get_module(dct['script']['path'])
    #del dct['script']
    dct.update(module.plot({}))
    

def create_object(msgs, bag_times, input_field=''):
    if len(msgs) == 0:
        a = CustomArray([])
        a.bag_times = np.array([])
        return a
    
    first = msgs[0]

    #print('field', input_field, hasattr(first, '_fields_and_field_types'))
    if not hasattr(first, '_fields_and_field_types'):
        a = CustomArray(msgs)
        a.bag_times = bag_times
        return a

    obj = type('', (), {})
    fields = first.get_fields_and_field_types()

    for field in fields:
        values = [getattr(msg, field) for msg in msgs]
        o = create_object(values, bag_times, field)
        setattr(obj, field, o)
        if type(o) == CustomArray:
            custom_array_container.add_member(obj, field)
    return obj
        
def load_data(bag_path, data_map):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='', output_serialization_format='')
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    # auto detect namespace
    prefixes = [topic.name.split('/', 2)[1] if topic.name.count('/') >= 2 else topic.name.replace('/', '') for topic in topic_types ]
    s = set(prefixes)
    d = [(prefix, prefixes.count(prefix)/float(len(prefixes))) for prefix in s]
    namespace, frequency = max(d, key=lambda x:x[1])
    if frequency < 0.8:
        namespace = ''
    else:
        namespace = '/' + namespace

    topic_map = {}
    for name, d in data_map.items():
        topic = d['topic']
        if topic[0] != '/':
            topic = namespace + '/' + topic
        if topic not in topic_map.keys():
            topic_map[topic] = {'names': [], 'fields': []}
        topic_map[topic]['names'].append(name)
        topic_map[topic]['fields'].append(d['field'])

    data = {name:[] for name, _ in data_map.items()}
    data_bag_times = {name:[] for name, _ in data_map.items()}
    
    while reader.has_next():
        topic, msg_data, recorded_time = reader.read_next()
        if topic in topic_map.keys():
            msg = deserialize_message(msg_data, get_message(type_map[topic]))
            fields = topic_map[topic]['fields']
            names = topic_map[topic]['names']
            for i in range(len(fields)):
                field = fields[i]
                name = names[i]
                if field == '':
                    value = msg
                else:
                    value = eval('msg' + field)
                data[name].append(value)
                data_bag_times[name].append(recorded_time)


    keys_to_delete = []
    for k,v in data.items():
        if len(v) == 0:
            keys_to_delete.append(k)
        o = create_object(v, np.array(data_bag_times[k]), 'TOP ---------------------------------------- ' + k)
        if hasattr(o, 'header'):
            o.header_time = o.header.stamp.sec + o.header.stamp.nanosec*1e-9
            custom_array_container.add_member(o, 'header_time')
        data[k] = o

    for k in keys_to_delete:
        del data[k]
                
    return data

class PlotInput:
    def __init__(self):
        self.dct = {}

    def on_click(self, event):
        if not event.dblclick:
            return

        fullscreen_pos = [0.1, 0.1, 0.8, 0.8]
        
        fig = event.canvas.figure
        if fig not in self.dct:
            self.dct[fig] = {'fullscreen_ax': None, 'positions': [ax.get_position() for ax in fig.get_axes()]}

        if self.dct[fig]['fullscreen_ax'] == None:
            self.dct[fig]['fullscreen_ax'] = event.inaxes
            event.inaxes.set_position(fullscreen_pos)
            for ax in fig.get_axes():
                if ax != event.inaxes:
                    ax.set_visible(False)
        else:
            self.dct[fig]['fullscreen_ax'] = None
            for i, ax in enumerate(fig.get_axes()):
                ax.set_position(self.dct[fig]['positions'][i])
                ax.set_visible(True)
        fig.canvas.draw_idle()
        
def plot(yaml_filename, bag_paths):
    with open(yaml_filename, 'r') as f:
        y = yaml.safe_load(f)
        figs = y['figures']
        d = y['data']
        rcparams = y['rcparams']
        profiles = y['profiles']
        def get_profile_context_function(k):
            def profile_context_function(**kwargs):
                return {'key': k, 'kwargs': kwargs}
            return profile_context_function
        profiles_context = {k:get_profile_context_function(k) for k in profiles}
        data_map = {}
        for name, text in d.items():
            dot_split = text.split('.', 1)
            topic = dot_split[0]
            if len(dot_split) > 1:
                field = '.' + text.split('.', 1)[1]
            else:
                field = ''
            data_map[name] = {'topic': topic, 'field': field}
    
    custom_context = {}
    for bag_path in bag_paths:
        data = load_data(bag_path, data_map)
        for k, v in data.items():
            if k not in custom_context:
                custom_context[k] = v
                if type(v) == CustomArray:
                    custom_array_container.add_standalone(k)
        
    plt.rcParams.update(rcparams)

    plot_input = PlotInput()

    for fig in figs:
        current_context = custom_context.copy()
        if 'profiles' in fig:
            # handle fig profiles
            try:
                temp_profiles_context = profiles_context.copy()
                temp_profiles_context.update(current_context)
                current_context.update(apply_profiles(fig, profiles, make_list(eval(fig['profiles'], globals(), temp_profiles_context))))
            except Exception as e:
                print('skipping ' + str(fig) + ' because ' + str(e))
                continue

        # configure splitting
        split_bag_times = [0]
        if 'split' in fig:
            for criteria in fig['split']['critera']:
                try:
                    field = eval(criteria['field'], globals(), current_context)
                    for i, x in enumerate(field):
                        # TODO don't allow x, i to be used as field names in data section
                        current_context['x'] = x
                        current_context['i'] = i
                        if eval(criteria['condition'], globals(), current_context):
                            split_bag_times.append(field.bag_times[i])
                except Exception as e:
                    print('skipping split field "' + criteria['field'] + '" because ' + str(e))
        split_bag_times.append(10**100)
        split_bag_times = np.array(sorted(split_bag_times))

        # figure out whether to split as figures or axes
        split_as_figures = True
        if 'split' in fig:
            if 'as' in fig['split']:
                if fig['split']['as'] not in ['figures', 'axes']:
                    print('warning: for split, the as field must be either figures or axes not ' + fig['split']['as'] + \
                          '. defaulting to figures.')
                else:
                    split_as_figures = fig['split']['as'] == 'figures'

        # choose which splits to keep
        if 'keep_indices' in fig['split']:
            keep_indices = np.array(make_list(eval(str(fig['split']['keep_indices']))))
            keep = list(set(np.append(keep_indices, keep_indices+1)))
            split_bag_times = split_bag_times[keep]
            if 'remove_indices' in fig['split']:
                print('warning: can\'t do keep_indices and remove_indices in same split. only using keep_indices.')
        elif 'remove_indices' in fig['split']:
            remove = make_list(eval(str(fig['split']['remove_indices'])))
            keep = [x for x in range(len(split_bag_times)) if x not in remove]
            split_bag_times = split_bag_times[keep]
        del fig['split']

        # check if splitting can be done as axes
        if not split_as_figures:
            for ax in fig['axes']:
                total_rows, total_cols = get_row_col_info(fig['axes'])
                if total_rows != 1 and total_cols != 1:
                    split_as_figures = True
                    print('can\'t split as axes, reverting to splitting as figures for ' + str(fig))
                    break
        
        if not split_as_figures:
            f = plt.figure()
            f.canvas.mpl_connect('button_press_event', plot_input.on_click)
        for split_index in range(1, len(split_bag_times)):
            if split_as_figures:
                f = plt.figure()
                f.canvas.mpl_connect('button_press_event', plot_input.on_click)
            
            start_time = split_bag_times[split_index-1]
            end_time = split_bag_times[split_index]
            
            if split_as_figures:
                custom_array_container.slice(start_time, end_time, current_context)
            
            # call functions
            for k in fig:
                if k not in ['axes', 'profiles']:
                    apply_function(f, k, fig[k], current_context)
            
            if not split_as_figures:
                custom_array_container.slice(start_time, end_time, current_context)

            total_rows, total_cols = get_row_col_info(fig['axes'])

            for ax in fig['axes']:
                # handle ax profiles
                ax_context = current_context.copy()
                if 'profiles' in ax:
                    temp_profiles_context = profiles_context.copy()
                    temp_profiles_context.update(current_context)
                    ax_context.update(apply_profiles(ax, profiles, make_list(eval(ax['profiles'], globals(), temp_profiles_context))))

                # configure subplots
                if not split_as_figures:
                    if total_rows == 1:
                        split_total_rows = len(split_bag_times)-1
                        split_total_cols = total_cols
                        row = split_index
                        col = ax['col']
                    else:
                        split_total_rows = total_rows
                        split_total_cols = len(split_bag_times)-1
                        row = ax['row']
                        col = split_index
                    a = f.add_subplot(*get_subplot_params(split_total_rows, split_total_cols, row, col))
                else:
                    a = f.add_subplot(*get_subplot_params(total_rows, total_cols, ax['row'], ax['col']))

                # call functions
                for k in ax:
                    if k not in ['plots', 'row', 'col', 'profiles']:
                        apply_function(a, k, ax[k], ax_context)

                # plotting
                for plot in ax['plots']:
                    handle_scripts(plot)
                    if 'script' in plot:
                        continue
                    
                    args = []
                    # configure x
                    if 'x' in plot.keys():
                        try:
                            x = eval(plot['x'], globals(), ax_context)
                        except Exception as e:
                            print(f'error in x ({plot["x"]}): {e}. skipping {plot}')
                            continue
                        args.append(x)
                        
                    # configure y
                    try:
                        y = eval(plot['y'], globals(), ax_context)
                    except Exception as e:
                        print(f'error in y ({plot["y"]}): {e}. skipping {plot}')
                        continue
                    args.append(y)

                    # configure args
                    if 'fmt' in plot.keys():
                        args.append(eval("'" + plot['fmt'] + "'"))
                    kwargs = {k:eval("'" + v + "'") for k,v in plot.items() if (k not in ['x', 'y', 'fmt', 'script'])}

                    # plot
                    a.plot(*args, **kwargs)
                    if 'label' in kwargs:
                        a.legend()
            custom_array_container.unslice(current_context)
    plt.show()
        
if __name__ == "__main__":
    yaml_filename = sys.argv[1]
    bag_paths = sys.argv[2:]
    plot(yaml_filename, bag_paths)

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

def get_time(stamp):
    return stamp.sec + stamp.nanosec*1e-9

class CustomArray(np.ndarray):
    def __new__(cls, input_array):
        obj = np.asarray(input_array).view(cls)
        obj.bag_times = None
        return obj

    def __array_finalize__(self, obj):
        if obj is None:
            return
        self.bag_times = getattr(obj, 'bag_times', None)

def make_list(o):
    if type(o) in [tuple, set]:
        return list(o)
    else:
        return [o]

def get_data_and_bag_times(s, extra_context={}):
    x = eval(s, globals(), extra_context)
    if hasattr(x, 'bag_times'):
        return x, x.bag_times
    elif type(x) == tuple:
        return x[0], x[1].bag_times
    else:
        return x, None

def apply_function(variable, function_name, function_arguments, extra_context={}):
    if function_name not in dir(variable):
        print('warning: ' + function_name + ' not a member of ' + str(type(variable)) + '. skipping.')
        return
    try:
        context = {'variable': variable}
        context.update(extra_context)
        eval('variable.' + function_name + '(' + function_arguments + ')', globals(), context)
    except:
        context = {'variable': variable}
        context.update(extra_context)
        eval('variable.' + function_name + '(\'' + function_arguments + '\')', globals(), context)

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

def load_functions(functions):
    context = {}
    for name, dct in functions.items():
        def get_func():
            def func(**kwargs):
                pass
            return func
        context[name] = get_func()

def create_object(msgs, bag_times):
    if len(msgs) == 0:
        a = CustomArray([])
        a.bag_times = np.array([])
        return a
    
    first = msgs[0]

    if not hasattr(first, '_fields_and_field_types'):
        a = CustomArray(msgs)
        a.bag_times = bag_times
        return a

    obj = type('', (), {})
    fields = first.get_fields_and_field_types()

    for field in fields:
        values = [getattr(msg, field) for msg in msgs]
        o = create_object(values, bag_times)
        setattr(obj, field, o)
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

    for k,v in data.items():
        o = create_object(v, np.array(data_bag_times[k]))
        if hasattr(o, 'header'):
            o.header_time = o.header.stamp.sec + o.header.stamp.nanosec*1e-9
        data[k] = o
                
    return data
            
        
def plot(yaml_filename, bag_path):
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
        #for k, v in profiles_context.items():
        #    print('key', k)
        #    print('value', v())
        data_map = {}
        for name, text in d.items():
            dot_split = text.split('.', 1)
            topic = dot_split[0]
            if len(dot_split) > 1:
                field = '.' + text.split('.', 1)[1]
            else:
                field = ''
            data_map[name] = {'topic': topic, 'field': field}
    
    data = load_data(bag_path, data_map)
    for k, v in data.items():
        exec('global ' + k + '\n' + k + ' = data[k]', globals(), locals())
        
    plt.rcParams.update(rcparams)

    for fig in figs:
        fig_context = {}
        if 'profiles' in fig:
            fig_context = apply_profiles(fig, profiles, make_list(eval(fig['profiles'], globals(), profiles_context)))

        # configure splitting
        split_bag_times = [0]
        if 'split' in fig:
            field = eval(fig['split']['field'])
            for i, x in enumerate(field):
                if eval(fig['split']['condition']):
                    split_bag_times.append(field.bag_times[i])
        split_bag_times.append(10**100)
        split_bag_times = np.array(split_bag_times)

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
        for split_index in range(1, len(split_bag_times)):
            if split_as_figures:
                f = plt.figure()
            
            start_time = split_bag_times[split_index-1]
            end_time = split_bag_times[split_index]

            # call functions
            for k in fig:
                if k not in ['axes', 'profiles']:
                    apply_function(f, k, fig[k], fig_context)

            total_rows, total_cols = get_row_col_info(fig['axes'])

            for ax in fig['axes']:
                # handle profiles
                ax_context = fig_context.copy()
                if 'profiles' in ax:
                    ax_context.update(apply_profiles(ax, profiles, make_list(eval(ax['profiles'], globals(), profiles_context))))

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
                    args = []
                    # configure x
                    if 'x' in plot.keys():
                        x, x_bag_times = get_data_and_bag_times(plot['x'], ax_context)
                        try:
                            x = x[np.where(np.logical_and(x_bag_times >= start_time, x_bag_times <= end_time))]
                        except Exception as e:
                            #print(e)
                            pass
                        args.append(x)
                        
                    # configure y
                    y, y_bag_times = get_data_and_bag_times(plot['y'], ax_context)
                    try:
                        y = y[np.where(np.logical_and(y_bag_times >= start_time, y_bag_times <= end_time))]
                    except Exception as e:
                        #print(e)
                        pass
                    args.append(y)

                    # configure args
                    if 'fmt' in plot.keys():
                        args.append(eval("'" + plot['fmt'] + "'"))
                    kwargs = {k:eval("'" + v + "'") for k,v in plot.items() if (k != 'x' and k != 'y' and k != 'fmt')}

                    # plot
                    a.plot(*args, **kwargs)
                    if 'label' in kwargs:
                        a.legend()
                
    plt.show()
        
if __name__ == "__main__":
    yaml_filename = sys.argv[1]    
    for i in range(2, len(sys.argv)):
        bag_path = sys.argv[i]
        print(bag_path)
        plot(yaml_filename, bag_path)

import argparse
import torch


def get_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Multi-Agent-Semantic-Exploration')

    # General Arguments
    parser.add_argument('--seed', type=int, default=1,
                        help='random seed (default: 1)')
    # Logging, loading models, visualization
    parser.add_argument('--log_interval', type=int, default=10,
                        help="""log interval, one log per n updates
                                (default: 10) """)
    parser.add_argument('-d', '--dump_location', type=str, default="./tmp",
                        help='path to dump models and log (default: ./tmp/)')
    parser.add_argument('--exp_name', type=str, default="exp1",
                        help='experiment name (default: exp1)')
    parser.add_argument('-v', '--visualize', type=int, default=0,
                        help="""1: Render the observation and
                                   the predicted semantic map
                                (default: 0)""")
    parser.add_argument('--print_images', type=int, default=0,
                        help='1: save visualization as images')

    # Environment, dataset and episode specifications
    parser.add_argument('-fw', '--frame_width', type=int, default=640,
                        help='Frame width (default:160)')
    parser.add_argument('-fh', '--frame_height', type=int, default=480,
                        help='Frame height (default:120)')
    parser.add_argument("--task_config", type=str,
                        default="multi_objectnav_hm3d.yaml",
                        help="path to config yaml containing task information")
    parser.add_argument('--hfov', type=float, default=79.0,
                        help="horizontal field of view in degrees")

    # Model Hyperparameters
    parser.add_argument('--agent', type=str, default="sem_exp")
    parser.add_argument('--num_local_steps', type=int, default=25,
                        help="""Number of steps the local policy
                                between each global step""")
    parser.add_argument('-n', '--num_processes', type=int, default=1)
    parser.add_argument('--rank', type=int, default=0)
    parser.add_argument('--gpu_id', type=int, default=0)

    parser.add_argument('--map_resolution', type=int, default=5)
    parser.add_argument('--map_size_cm', type=int, default=2400)
    parser.add_argument('--map_height_cm', type=int, default=130)
    parser.add_argument('--sem_threshold', type=float, default=0.85)
    parser.add_argument('--num_agents', type=int, default=2)
    
    
    # train_se_frontier
    parser.add_argument('--nav_mode', type=str, default="gpt",
                        choices=['nearest', 'co_ut', 'fill', "gpt"])
    parser.add_argument('--fill_mode', type=int, default=0)
    parser.add_argument('--gpt_type', type=int, default=2,
                        help="""0: text-davinci-003
                                1: gpt-3.5-turbo
                                2: gpt-4o
                                3: gpt-4o-mini
                                (default: 2)""")
                                   
    # parse arguments
    args = parser.parse_args(argv)

    args.cuda = torch.cuda.is_available()

    return args

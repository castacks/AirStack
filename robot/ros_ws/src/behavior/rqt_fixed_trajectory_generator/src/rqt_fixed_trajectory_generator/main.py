import sys

from rqt_gui.main import Main


def main():
    main = Main()
    sys.exit(main.main(sys.argv, standalone='rqt_py_console.py_console.PyConsole'))


if __name__ == '__main__':
    main()

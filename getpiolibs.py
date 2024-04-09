#!/usr/bin/env python3

'''
pio install libs globally from platformio.ini
'''

import argparse
import configparser
import sys
import subprocess
from pprint import pprint


LIB_DEPS = 'lib_deps'
DEFAULT_SECTION = 'platformio'
DEFAULT_SECTION_ENVS = 'default_envs'
ENV_PREFIX = 'env:'
PIO_COMMAND = 'pio'
SKIP_BUILTIN = [
    'Wire',
]


def get_pio_libs(envsetction) -> list:
    ''' Get the libraries of specified env'''
    libs = envsetction.get(LIB_DEPS, None)
    if not libs:
        return []
    return libs.split()


def find_libs(cfg: configparser.ConfigParser, names: list):
    ''' get set of libs in sections '''
    allibs = set()
    for name in names:
        print('Adding env:', name)
        secname = ENV_PREFIX + name
        if secname not in cfg.sections():
            print(f'Warning: "{name}" environment not found!', file=sys.stderr)
            continue
        libs = get_pio_libs(cfg[secname])
        allibs = set([*allibs, *libs])
    return allibs


def argv_parse():
    ''' parse system argv for the script '''
    parser = argparse.ArgumentParser(
        prog='getpiolibs',
        description='Install libs of pio env globally'
    )
    parser.add_argument('-e', '--environment',
        dest='env', required=False,
        help='Environment e.g. from [env:exampleboard]')
    parser.add_argument('-s', '--skip',
        dest='skip', required=False,
        help='in bash use quotes with space between " " for none ')
    return parser.parse_args()


def main():
    ''' Entrypoint '''
    args = argv_parse()

    cfg = configparser.ConfigParser(inline_comment_prefixes=('#', ';'))
    cfg.read('platformio.ini')
    libs = set()
    if args.env:
        libs = find_libs(cfg, [args.env])
    else:
        print('Assuming default env')
        if DEFAULT_SECTION not in cfg.sections():
            print(f'Error: "{DEFAULT_SECTION}" section not found' , file=sys.stderr)
            return -1
        pio_sec = cfg[DEFAULT_SECTION]
        default_envs = pio_sec.get(DEFAULT_SECTION_ENVS, None)
        if not isinstance(default_envs, str):
            print(f'Error: "{DEFAULT_SECTION_ENVS}" not found' , file=sys.stderr)
            return -1
        envs = [ e.strip() for e in default_envs.split(',') if e.strip() ]
        libs = find_libs(cfg, envs)

    params = [ PIO_COMMAND, 'pkg', 'install', '--global' ]
    to_skip = []
    for lib in  libs:
        to_skip = SKIP_BUILTIN
        if args.skip:
            to_skip = args.skip.split()
        if lib in to_skip:
            continue
        params.append('--library')
        params.append(lib)
    print('Installing libs:')
    pprint([*libs])
    print('Omitting:', to_skip)

    try:
        subprocess.run(params, shell=False, check=True)
    except FileNotFoundError:
        print(f'Error: "{PIO_COMMAND}" seems missing in PATH', file=sys.stderr)
        return -1
    except subprocess.CalledProcessError:
        print(f'Error: install command "{params}" failed', file=sys.stderr)
        return -1
    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)

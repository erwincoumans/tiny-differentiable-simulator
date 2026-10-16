"""Command line of tds_bake.

  python -m tds_bake schema                  the JSON description of worlds
  python -m tds_bake example g1_walk > w.json
  python -m tds_bake bake w.json --backend cuda --precision float
  python -m tds_bake info w.json             bodies, joints, contacts, kernels
  python -m tds_bake bench w.json --num-worlds 1024 4096
  python -m tds_bake view w.json --num-worlds 256     (needs pytinyopengl3)
  python -m tds_bake view w.json --num-worlds 4096 --mode scene
  python -m tds_bake list                    the baked worlds in the cache
  python -m tds_bake clean [name]            removes baked worlds
"""

import argparse
import json
import os
import sys
import time

from . import baking, examples
from .world import SCHEMA_DOC, World


def _load_world(path):
  if os.path.isdir(path):
    return None, path
  if path in examples.EXAMPLES:
    return examples.EXAMPLES[path](), None
  return World.load(path), None


def _baked(args, compile=True):
  world, directory = _load_world(args.world)
  if directory is not None:
    return baking.load(directory)
  backends = args.backend if compile else []
  precisions = args.precision if compile else []
  return baking.bake(world, backends=backends, precisions=precisions,
                     force=getattr(args, "force", False),
                     max_segment_kb=getattr(args, "max_segment_kb", 256),
                     jobs=getattr(args, "jobs", None))


def cmd_schema(args):
  print(SCHEMA_DOC)


def cmd_example(args):
  if args.name not in examples.EXAMPLES:
    sys.exit("unknown example %r, choose from %s" %
             (args.name, ", ".join(examples.EXAMPLES)))
  print(examples.EXAMPLES[args.name]().to_json())


def cmd_bake(args):
  start = time.time()
  baked = _baked(args)
  print(baked.describe())
  print("baked %r in %.1f s: %s" % (baked.name, time.time() - start,
                                    baked.directory))


def cmd_info(args):
  baked = _baked(args, compile=False)
  print(baked.describe())
  print("directory: %s" % baked.directory)
  if args.json:
    print(json.dumps(baked.layout, indent=2))


def cmd_bench(args):
  baked = _baked(args)
  for backend in args.backend:
    for precision in args.precision:
      for n in args.num_worlds:
        batch = baked.batch(n, backend, precision)
        batch.step()
        batch.synchronize()
        start = time.time()
        for _ in range(args.frames):
          batch.step()
        batch.synchronize()
        seconds = (time.time() - start) / args.frames
        steps = n * batch.substeps / seconds
        print("%-8s %-6s %6d worlds: %8.3f ms per step() (%d substeps), "
              "%.3g world-steps/s, %.0fx real time" %
              (backend, precision, n, 1e3 * seconds, batch.substeps, steps,
               n * batch.substeps * batch.dt / seconds))


def cmd_view(args):
  from .render import view
  baked = _baked(args)
  view(baked, num_worlds=args.num_worlds, backend=args.backend[0],
       precision=args.precision[0], mode=args.mode,
       tiles=(args.tiles_x, args.tiles_y), num_shown=args.num_shown,
       spacing=args.spacing, shadows=not args.no_shadows)


def cmd_list(args):
  worlds = baking.cached_worlds()
  if not worlds:
    print("no baked worlds in %s" % baking.cache_root())
  for w in worlds:
    print("%-24s %7.1f MB  %-40s %s" % (w["name"], w["megabytes"],
                                         ", ".join(w["compiled"]) or "-",
                                         w["directory"]))


def cmd_clean(args):
  if args.name is None and not args.all:
    sys.exit("give the name of a world, or --all")
  for d in baking.clear_cache(args.name):
    print("removed " + d)


def main(argv=None):
  parser = argparse.ArgumentParser(
      prog="python -m tds_bake",
      description="Bake worlds into batched CUDA and CPU kernels.",
      formatter_class=argparse.RawDescriptionHelpFormatter,
      epilog=__doc__)
  sub = parser.add_subparsers(dest="command", required=True)

  def world_command(name, func, help, compile=True):
    p = sub.add_parser(name, help=help)
    p.add_argument("world", help="world JSON file, example name, or the "
                   "directory of a baked world")
    if compile:
      p.add_argument("--backend", nargs="+", default=["cuda"],
                     choices=["cuda", "openmp", "serial"])
      p.add_argument("--precision", nargs="+", default=["float"],
                     choices=["float", "double"])
      p.add_argument("--jobs", type=int, default=None,
                     help="parallel compile jobs (default: CPUs and memory)")
    p.set_defaults(func=func)
    return p

  p = sub.add_parser("schema", help="print the description of world JSON")
  p.set_defaults(func=cmd_schema)
  p = sub.add_parser("example", help="print an example world as JSON")
  p.add_argument("name", help=", ".join(examples.EXAMPLES))
  p.set_defaults(func=cmd_example)
  p = world_command("bake", cmd_bake, "generate and compile the kernels")
  p.add_argument("--force", action="store_true",
                 help="generate again, also when cached")
  p.add_argument("--max-segment-kb", type=int, default=256)
  p = world_command("info", cmd_info, "describe a world", compile=False)
  p.add_argument("--json", action="store_true", help="print the layout")
  p = world_command("bench", cmd_bench, "measure the simulation speed")
  p.add_argument("--num-worlds", type=int, nargs="+", default=[1024])
  p.add_argument("--frames", type=int, default=100)
  p = world_command("view", cmd_view, "simulate and render a batch of worlds")
  p.add_argument("--num-worlds", type=int, default=64)
  p.add_argument("--mode", default="tiles", choices=["tiles", "scene"],
                 help="tiles: one world per tile; scene: all worlds side by "
                 "side in one 3D view")
  p.add_argument("--tiles-x", type=int, default=8)
  p.add_argument("--tiles-y", type=int, default=8)
  p.add_argument("--num-shown", type=int, default=None,
                 help="worlds in the scene (default: all)")
  p.add_argument("--spacing", type=float, default=None,
                 help="distance of the worlds in the scene (meters)")
  p.add_argument("--no-shadows", action="store_true",
                 help="no shadow pass, faster with many worlds")
  p = sub.add_parser("list", help="list the baked worlds in the cache")
  p.set_defaults(func=cmd_list)
  p = sub.add_parser("clean", help="remove baked worlds from the cache")
  p.add_argument("name", nargs="?")
  p.add_argument("--all", action="store_true")
  p.set_defaults(func=cmd_clean)

  args = parser.parse_args(argv)
  args.func(args)


if __name__ == "__main__":
  main()

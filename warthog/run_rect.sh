#!/usr/bin/bash

maps=(
  # ../maps/dao/arena.map
  ../maps/iron/scene_sp_endmaps.map
  ../maps/starcraft/GreenerPastures.map
  ../maps/starcraft/CatwalkAlley.map
)

scens=(
  # ../scenarios/movingai/dao/arena.map.scen
  ../scenarios/movingai/iron/scene_sp_endmaps.map.scen
  ../scenarios/movingai/starcraft/GreenerPastures.map.scen
  ../scenarios/movingai/starcraft/CatwalkAlley.map.scen
)

algs=(rect jps2 jps2+)
exec="./build/fast/bin/warthog"

function exp() {
  for (( j=0; j<${#maps[@]}; j++ )); do
    name=$(basename ${maps[$j]} .map)
    for i in {1..8}; do
      spath="./data/${name}_$i.map.scen"
      mpath="./data/${name}_$i.map"
      for alg in "${algs[@]}"; do
        outpath="./output/$alg"
        mkdir -p $outpath
        cmd="$exec --scen ${spath} --map ${mpath} --alg $alg > $outpath/${name}_$i.log"
        echo $cmd
        eval "$cmd"
      done
    done
  done
}

function gen() {
  for (( j=0; j<${#maps[@]}; j++ )); do
    map=${maps[$j]}
    scen=${scens[$j]}
    name=$(basename $map .map)
    for i in {1..8}; do
      cmd="./scripts/gen_scalemap.py scale $map $i > data/${name}_$i.map"
      echo $cmd
      eval $cmd
      cmd="./scripts/gen_scalemap.py scale-scen $scen $i > data/${name}_$i.map.scen"
      echo $cmd
      eval $cmd
    done
  done
}

function clean() {
  rm -f data/*.jps+
  rm -rf output
}

case "$1" in
  exp) exp ;;
  cexp) clean && exp ;;
  gen) gen ;;
  clean) clean ;; 
  *)
    echo $"Usage: $0 {exp|gen}"
    exit 1
esac

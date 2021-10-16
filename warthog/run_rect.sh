#!/usr/bin/bash
maps=(
  ./data/endmaps_1.map
  ./data/endmaps_2.map
  ./data/endmaps_3.map
  ./data/endmaps_4.map
  ./data/endmaps_5.map
  ./data/endmaps_6.map
  ./data/endmaps_7.map
  ./data/endmaps_8.map
)

scens=(
  ./data/endmaps_1.map.scen
  ./data/endmaps_2.map.scen
  ./data/endmaps_3.map.scen
  ./data/endmaps_4.map.scen
  ./data/endmaps_5.map.scen
  ./data/endmaps_6.map.scen
  ./data/endmaps_7.map.scen
  ./data/endmaps_8.map.scen
)

algs=(rect jps2)
exec="./build/fast/bin/warthog"

function exp() {
  for (( i=0; i<${#maps[@]}; i++ )); do
    mpath=${maps[$i]}
    spath=${scens[$i]}
    mapname=$(basename -- $mpath)
    for alg in "${algs[@]}"; do
      outpath="./output/$alg"
      mkdir -p $outpath
      cmd="$exec --scen ${spath} --map ${mpath} --alg $alg > $outpath/$mapname.log"
      echo $cmd
      eval "$cmd"
    done
  done
}

function gen() {
  for i in {1..8}; do
    cmd="./scripts/gen_scalemap.py scale ../maps/iron/scene_sp_endmaps.map $i > data/endmaps_$i.map"
    echo $cmd
    eval $cmd
    cmd="./scripts/gen_scalemap.py scale-scen ../scenarios/movingai/iron/scene_sp_endmaps.map.scen $i > data/endmaps_$i.map.scen"
    echo $cmd
    eval $cmd
  done
}

case "$1" in
  exp) exp ;;
  gen) gen ;;
  *)
    echo $"Usage: $0 {exp|gen}"
    exit 1
esac

#!/usr/bin/bash

maps=(
  # ../maps/dao/arena.map
  ../maps/iron/scene_sp_endmaps.map
  # ../maps/iron/scene_sp_cha_02.map
  # ../maps/street/Denver_2_1024.map
  # ../maps/bgmaps/AR0044SR.map
  ../maps/starcraft/GreenerPastures.map
  ../maps/starcraft/CatwalkAlley.map
  # ../maps/street/Boston_0_256.map
  ../maps/maze512/maze512-32-0.map
  ../maps/rooms/64room_000.map
)

scens=(
  # ../scenarios/movingai/dao/arena.map.scen
  ../scenarios/movingai/iron/scene_sp_endmaps.map.scen
  # ../scenarios/movingai/iron/scene_sp_cha_02.map.scen
  # ../scenarios/movingai/street/Denver_2_1024.map.scen
  # ../scenarios/movingai/bgmaps/AR0044SR.map.scen
  ../scenarios/movingai/starcraft/GreenerPastures.map.scen
  ../scenarios/movingai/starcraft/CatwalkAlley.map.scen
  # ../scenarios/movingai/street/Boston_0_256.map.scen
  ../scenarios/movingai/maze512/maze512-32-0.map.scen
  ../scenarios/movingai/rooms/64room_000.map.scen
)

domains=(
  bgmaps dao iron maze512 random10 starcraft street
)

algs=(jps2 jps)
exec="./build/fast/bin/warthog"


function gen_dimacs() {
  maps=(
    # ./data/GreenerPastures_1.map
    # ./data/GreenerPastures_2.map
    ./data/GreenerPastures_8.map

    # ./data/scene_sp_cha_02_1.map
    # ./data/scene_sp_cha_02_2.map
    ./data/scene_sp_cha_02_8.map

    # ./data/64room_000_1.map
    # ./data/64room_000_2.map
    ./data/64room_000_8.map

    # ./data/maze512-32-0_1.map
    # ./data/maze512-32-0_2.map
    ./data/maze512-32-0_8.map

    # ./data/Denver_2_1024_1.map
    # ./data/Denver_2_1024_2.map
    ./data/Denver_2_1024_8.map

    # ./data/AR0044SR_1.map
    # ./data/AR0044SR_2.map
    ./data/AR0044SR_8.map
  )
  for map in "${maps[@]}"; do
    name=$(basename ${map} .map)
    scen="./data/${name}.map.scen"
    cmap="./data/changed/${name}.map"
    cfile="./data/${name}.repair"
    dimacsdir="./data/dimacs/${name}"
    mkdir -p $dimacsdir
    grfile="$dimacsdir/${name}.gr"
    cofile="$dimacsdir/${name}.co"
    qfile="$dimacsdir/${name}.query"
    dfile="$dimacsdir/${name}.diff"
    grfile2="$dimacsdir/${name}_changed.gr"
    cofile2="$dimacsdir/${name}_changed.co"
    # cmd="./scripts/grid2dimacs.py $map $scen $cfile $grfile $qfile $dfile"
    cmd="./build/fast/bin/grid2graph $map $scen $cfile $grfile $cofile $qfile $dfile $grfile2 $cofile2"
    echo $cmd
    eval $cmd

    cp -r $dimacsdir "$HOME/projects/RoutingKit/data/bundles/"
  done
}

function jpsp_vs_rect() {
  maps=(
    # ./data/CatwalkAlley_1.map
    # ./data/CatwalkAlley_2.map
    # ./data/CatwalkAlley_8.map
    ./data/GreenerPastures_1.map
    ./data/GreenerPastures_2.map
    ./data/GreenerPastures_8.map
    # ./data/scene_sp_endmaps_1.map
    # ./data/scene_sp_endmaps_2.map
    # ./data/scene_sp_endmaps_8.map
    ./data/scene_sp_cha_02_1.map
    ./data/scene_sp_cha_02_2.map
    ./data/scene_sp_cha_02_8.map
    ./data/64room_000_1.map
    ./data/64room_000_2.map
    ./data/64room_000_8.map
    ./data/maze512-32-0_1.map
    ./data/maze512-32-0_2.map
    ./data/maze512-32-0_8.map
    ./data/Denver_2_1024_1.map
    ./data/Denver_2_1024_2.map
    ./data/Denver_2_1024_8.map
    ./data/AR0044SR_1.map
    ./data/AR0044SR_2.map
    ./data/AR0044SR_8.map
  )
  for map in "${maps[@]}"; do
    name=$(basename ${map} .map)
    scen="./data/${name}.map.scen"
    cmap="./data/changed/${name}.map"
    rfile="./data/${name}.rectid"
    crfile="./data/changed/${name}.rectid"

    outpath="./output/rect"
    mkdir -p $outpath
    cmd="$exec --scen ${scen} --map ${rfile} --alg rect > $outpath/${name}.log"
    echo $cmd
    eval $cmd

    outpath="./output/jpsp"
    mkdir -p $outpath
    cmd="$exec --scen ${scen} --map ${map} --alg jps+ > $outpath/${name}.log"
    eval $cmd
    echo $cmd

    outpath="./output/changed/rect"
    mkdir -p $outpath
    cmd="$exec --scen ${scen} --map ${crfile} --alg rect > $outpath/${name}.log"
    echo $cmd
    eval $cmd

    outpath="./output/changed/jpsp"
    mkdir -p $outpath
    cmd="$exec --scen ${scen} --map ${cmap} --alg jps+ > $outpath/${name}.log"
    eval $cmd
    echo $cmd
  done
}

function exp() {
  for (( j=0; j<${#maps[@]}; j++ )); do
    name=$(basename ${maps[$j]} .map)
    for i in {1..8}; do
      spath="./data/${name}_$i.map.scen"
      mpath="./data/${name}_$i.map"
      rpath="./data/${name}_$i.rectid"
      outpath="./output/rect"
      mkdir -p $outpath
      cmd="$exec --scen ${spath} --map ${rpath} --alg rect > $outpath/${name}_$i.log"
      echo $cmd
      eval $cmd

      # rpath="./data/${name}_$i.convrectid"
      # outpath="./output/convrect"
      # mkdir -p $outpath
      # cmd="$exec --scen ${spath} --map ${rpath} --alg convrect > $outpath/${name}_$i.log"
      # echo $cmd
      # eval $cmd
      #
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

function exp_domain() {
  for domain in ${domains[@]}; do
    for mpath in `ls ../maps/${domain}/*.map`; do
      name=$(basename ${mpath} .map)
      for i in {1..8}; do
        spath="./data/${name}_$i.map.scen"
        mpath="./data/${name}_$i.map"
        rpath="./data/${name}_$i.rectid"
        outpath="./output/rect"
        mkdir -p $outpath
        cmd="$exec --scen ${spath} --map ${rpath} --alg rect > $outpath/${name}_$i.log"
        echo $cmd
        eval $cmd

        for alg in "${algs[@]}"; do
          outpath="./output/$alg"
          mkdir -p $outpath
          cmd="$exec --scen ${spath} --map ${mpath} --alg $alg > $outpath/${name}_$i.log"
          echo $cmd
          eval $cmd
        done
      done
    done
  done
}

function gen_domain_scale() {
  domain=$1
  map=$2
  name=$(basename $map .map)
  scen="../scenarios/movingai/${domain}/${name}.map.scen"
  for i in {1..8}; do
    cmd="./scripts/gen_scalemap.py scale $map $i > data/${name}_$i.map"
    echo $cmd
    eval $cmd
    cmd="./scripts/gen_scalemap.py scale-scen $scen $i > data/${name}_$i.map.scen"
    echo $cmd
    eval $cmd
  done
}

function gen_rects() {
  for mpath in `ls data/*.map`; do
    rpath="${mpath%%.*}.rectid"
    cmd="./build/fast/test/rect_jps gen-rectid --in ${mpath} --out ${rpath}"
    if [[ ! -e ${rpath} ]]; then
      echo $cmd
      eval $cmd
    fi
  done

  # for mpath in `ls data/changed/*.map`; do
  #   rpath="${mpath%%.*}.rectid"
  #   cmd="./build/fast/test/rect_jps gen-rectid --in ${mpath} --out ${rpath}"
  #   if [[ ! -e ${rpath} ]]; then
  #     echo $cmd
  #     eval $cmd
  #   fi
  # done
}

function gen_all() {
  for domain in ${domains[@]}; do
    for mpath in `ls ../maps/${domain}/*.map`; do
      gen_domain_scale $domain $mpath
    done
  done
  gen_rects
}

function gen_all_small() {
  for i in {1..8}; do
    for mpath in ${maps[@]}; do
      name=$(basename $mpath .map)
      cmd="./scripts/gen_scalemap.py scale $mpath $i > data/${name}_$i.map"
      if [[ ! -e ${name}_$i.map ]]; then
        echo $cmd
        eval $cmd
      fi
    done
    for spath in ${scens[@]}; do
      name=$(basename $spath .map.scen)
      cmd="./scripts/gen_scalemap.py scale-scen $spath $i > data/${name}_$i.map.scen"
      if [[ ! -e ${name}_$i.map.scen ]]; then
        echo $cmd
        eval $cmd
      fi
    done
  done
  gen_rects
}

function gen_small_repair() {
  for i in {1..8}; do
    for mpath in ${mpath[@]}; do
      name=$(basename $mpath .map)
    done
  done
}

function clean() {
  rm -f data/*.jps+
  find output -name "*.log" -delete
}

case "$1" in
  cmp) jpsp_vs_rect ;;
  exp) exp ;;
  cexp) clean && exp ;;
  gen) gen_all_small ;;
  gen-rect) gen_rects ;;
  gen-dimacs) gen_dimacs ;;
  clean) clean ;; 
  *)
    echo $"Usage: $0 {exp|gen}"
    exit 1
esac

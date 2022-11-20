#!/bin/bash
domains=(
  ./maps/bgmaps
  ./maps/iron
  ./maps/random10
  ./maps/starcraft
  ./maps/street
  ./maps/dao
  ./maps/maze512
  ./maps/rooms
)
rand_domains=(
  ./maps-randomlized/bgmaps
  ./maps-randomlized/iron
  ./maps-randomlized/random10
  ./maps-randomlized/starcraft
  ./maps-randomlized/street
  ./maps-randomlized/dao
  ./maps-randomlized/maze512
  ./maps-randomlized/rooms
)
rs=(0.1 0.5 1 1.5)


algs=(
  jps2
  jps2-prune2
)
out_dir="./output2"

function run_domain() {
  domain=$1
  dname=$(basename -- $domain)
  for mpath in `ls ${domain}/*.map`; do
    mapname=$(basename -- $mpath)
    spath="./scenarios/movingai/${dname}/${mapname}.scen"

    for alg in "${algs[@]}"; do
      outpath="${out_dir}/$alg"
      mkdir -p ${outpath}
      cmd="./bin/warthog --scen ${spath} --map ${mpath} --alg $alg > $outpath/$mapname.log"
      echo $cmd
      eval "$cmd"
    done
  done
}


function exp() {
  for dm in ${domains[@]}; do
    run_domain $dm
  done
}

function suboptcnt_per_query_gen() {
  datasets=(
    "./maps/"
    "./data/maps-randomlized-0.1p"
    "./data/maps-randomlized-0.5p"
    "./data/maps-randomlized-1p"
    "./data/maps-randomlized-1.5p")

  outdirs=(
    "./exp-output/subcnt-0p"
    "./exp-output/subcnt-0.1p"
    "./exp-output/subcnt-0.5p"
    "./exp-output/subcnt-1p"
    "./exp-output/subcnt-1.5p"
  )

  for (( i=0; i<${#datasets[@]}; i++ )); do

    for domain in "${domains[@]}"; do
      dname=$(basename -- $domain)
      out_dir=${outdirs[$i]}
      for mpath in "${datasets[$i]}/${dname}"/*.map; do
        mapname=$(basename "$mpath" .map)
        spath="./scenarios/movingai/${dname}/${mapname}.map.scen"

        for alg in "${algs[@]}"; do
          outpath="${out_dir}/${dname}"
          mkdir -p "${outpath}"
          cmd="./build/fast/bin/experiment ${mpath} ${spath} subcnt > $outpath/$mapname.log"
          echo $cmd
        done
      done
    done

  done

}

function suboptcnt_per_query() {
  suboptcnt_per_query_gen > subcnt_per_query.sh
  chmod u+x subcnt_per_query.sh
  cat subcnt_per_query.sh | parallel 
}

function suboptcnt() {
  outdir="${out_dir}"
  fname="subopt-expd.log"
  mkdir -p ${outdir}
  if [[ -e ${outdir}/${fname} ]]; then
    rm ${outdir}/${fname}
  fi
  header=map'\t'subopt_touch'\t'tot_touch'\t'subopt_expd'\t'pruneable'\t'tot_expd'\t'scnt'\t'alg
  echo  -e "$header"> ${outdir}/${fname}
  # ./bin/subopt_expd_exp --scen data/subopt-expd.map.scen --map testcases/maps/subopt-expd.map >> ${outdir}/${fname}
  for dm in ${domains[@]}; do
    domain=$(basename -- $dm)
    for mpath in `ls ${dm}/*.map`; do
      mapname=$(basename -- $mpath)
      spath="./scenarios/movingai/${domain}/${mapname}.scen"
      cmd="./bin/subopt_expd_exp --scen ${spath} --map ${mpath} >> ${outdir}/${fname}"
      echo $cmd
      eval $cmd
    done
  done
}

function clean() {
  rm -f data/*.jps+
  fd -e log . "output" --no-ignore -x rm
  fd -e log . "small_output" --no-ignore -x rm
}


function genjobs() {
  rep=10
  for (( i=0; i<rep; i++ )) {
    for dm in "${rand_domains[@]}"; do
      domain=$dm
      dname=$(basename -- $domain)
      for mpath in `ls ${dm}/*.map`; do
        mapname=$(basename -- $mpath)
        spath="./scenarios/movingai/${dname}/${mapname}.scen"

        for alg in "${algs[@]}"; do
          outpath="${out_dir}/$alg/$i/"
          mkdir -p ${outpath}
          cmd="./build/fast/bin/warthog --scen ${spath} --map ${mpath} --alg $alg > ${outpath}/${mapname}.log"
          echo $cmd
        done
      done
    done
  }
}

function gen_rand() {
  for dm in "${domains[@]}"; do
    for map in `ls ${dm}/*.map`; do
      for r in "${rs[@]}"; do
        domain=$(basename ${dm})
        outpath="./maps-randomlized-${r}p/${domain}"
        mapname=$(basename ${map} .map)
        scenfile="./scenarios/movingai/${domain}/${mapname}.map.scen"
        if [[ ! -e ${scenfile} ]]; then
          echo "scenfile missing, map: ${map}"
          continue
        fi
        mkdir -p $outpath
        rval=$(python -c "print(${r} / 100.0)")
        cmd="./gen.py rand_scen ${map} ${scenfile} ${rval} > ${outpath}/${mapname}.map"
        echo $cmd
        eval $cmd
      done
    done
  done
}

case "$1" in
  exp) exp;;
  cexp) clean && exp;;
  genjobs) genjobs;;
  gen_rand) gen_rand ;;
  sub) suboptcnt;;
  subq) suboptcnt_per_query ;;
  clean) clean ;; 
  *)
    echo $"Usage: $0 {exp|gen}"
    exit 1
esac

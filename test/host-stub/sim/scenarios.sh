#!/bin/sh
# Run the scenario set against two simulator binaries, print the RESULT line of each.
#   test/host-stub/sim/scenarios.sh /tmp/sim-old.bin /tmp/fugu-sim [hours]
OLD=$1 NEW=$2 H=${3:-4}
D=$(mktemp -d)
run() { # name args...
    n=$1; shift
    "$OLD" "$@" hours=$H v=1 > "$D/$n.old" &
    "$NEW" "$@" hours=$H v=1 > "$D/$n.new" &
}
for isc in 0.08 0.10 0.12; do for k in 0.6 0.7 0.85; do for nz in 0 0.01 0.03; do
    run "isc${isc}_k${k}_n${nz}" isc=$isc voc=66 pv_k=$k vbat=25.6 noise_iout=$nz
done; done; wait; done
run dusk_0.01 isc=0.01 voc=66 pv_k=0.7 vbat=25.6
run dusk_0.01_n0.01 isc=0.01 voc=66 pv_k=0.7 vbat=25.6 noise_iout=0.01
run ramp_0.15-0.005_n0.01 isc=0.15 isc_end=0.005 voc=66 pv_k=0.7 vbat=25.6 noise_iout=0.01
run off-0.61_n0.01 isc=0.1 voc=66 pv_k=0.7 vbat=25.6 iout_off=-0.61 noise_iout=0.01
# dawn: Vin collapses to ~Vout (the model's input clamp stops ~9% above it, vin_k=0.9 stands in)
run dawn-collapse_0.01 isc=0.01 voc=66 pv_k=0.7 vbat=25.6 noise_iout=0.01 vin_k=0.9
# a lying current sensor in bright light must still trip
run iout-lies_3 isc=3 voc=66 pv_k=0.7 vbat=25.6 iout_k=0.01
wait
for f in "$D"/*.old; do
    n=$(basename "$f" .old)
    echo "$n"
    echo "  old $(grep RESULT "$f" | cut -c8-)"
    echo "  new $(grep RESULT "$D/$n.new" | cut -c8-)"
done
echo "logs: $D"

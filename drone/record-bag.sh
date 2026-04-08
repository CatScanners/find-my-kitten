#!/usr/bin/env -S bash -euo pipefail

if [[ $# -lt 1 ]]; then
	echo "Give file containing ros topics to record, newline separated" >&2
	exit 1
fi
topics_file=$1
shift

mapfile -t topics < "$topics_file"

GIT_TOP=$(git rev-parse --show-toplevel)
bags_dir="$GIT_TOP/flight/bags/$(date '+%F')/"
mkdir -p "$bags_dir"

bag_name="${topics_file##*/}_$(date '+%T')"

printf "%s\n" "${topics[@]}"
ros2 bag record -o "$bags_dir/$bag_name" "${topics[@]}"

# get version
version=$(git describe)

help_text="Usage: $0 <board> <sdkconfig> <chip> <bootloader_offset>"

# setup environment
board=$1
if [ "$board" = "" ]; then
    echo $help_text
    exit 1
fi
sdkconfig=$2
if [ "$sdkconfig" = "" ]; then
    echo $help_text
    exit 1
fi
if [ ! -f $sdkconfig ]; then
    echo "Error: $sdkconfig not found"
    exit 1
fi
chip=$3
if [ "$chip" = "" ]; then
    echo $help_text
    exit 1
fi
bootloader_offset=$4
if [ "$bootloader_offset" = "" ]; then
    echo $help_text
    exit 1
fi

echo "idf path:" $IDF_PATH

out=out
tmp=$out/$board
. $IDF_PATH/export.sh

# select sdkconfig
rm -f sdkconfig
rm -f sdkconfig.defaults
ln -s $sdkconfig sdkconfig.defaults

# determine partition table offsets from the selected partition table
partition_csv=$(grep 'CONFIG_PARTITION_TABLE_CUSTOM_FILENAME=' sdkconfig.defaults | sed 's/CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="\(.*\)"/\1/')
if [ -z "$partition_csv" ] || [ ! -f "$partition_csv" ]; then
    echo "Error: could not determine partition table CSV from $sdkconfig"
    exit 1
fi
factory_offset=$(grep '^factory,' "$partition_csv" | awk -F, '{print $4}')
storage_offset=$(grep '^storage,' "$partition_csv" | awk -F, '{print $4}')
if [ -z "$factory_offset" ] || [ -z "$storage_offset" ]; then
    echo "Error: could not extract factory/storage offsets from $partition_csv"
    exit 1
fi

# clean build
idf.py fullclean build || exit 1

# export package
rm -rf $tmp
mkdir -p $tmp

# copy image files
cp build/bootloader/bootloader.bin $tmp/
cp build/partition_table/partition-table.bin $tmp/
cp build/jaculus.bin $tmp/factory.bin
cp build/storage.bin $tmp/

# generate manifest.json
manifest=$tmp/manifest.json
echo "{" > $manifest
echo "    \"board\": \"$board\"," >> $manifest
echo "    \"version\": \"$version\"," >> $manifest
echo "    \"platform\": \"esp32\"," >> $manifest
echo "    \"config\": {" >> $manifest
echo "        \"flashBaud\": 921600," >> $manifest
echo "        \"chip\": \"$chip\"," >> $manifest
echo "        \"partitions\": [" >> $manifest
echo "            {" >> $manifest
echo "                \"name\": \"bootloader\"," >> $manifest
echo "                \"address\": \"$bootloader_offset\"," >> $manifest
echo "                \"file\": \"bootloader.bin\"" >> $manifest
echo "            }," >> $manifest
echo "            {" >> $manifest
echo "                \"name\": \"partition-table\"," >> $manifest
echo "                \"address\": \"0x8000\"," >> $manifest
echo "                \"file\": \"partition-table.bin\"" >> $manifest
echo "            }," >> $manifest
echo "            {" >> $manifest
echo "                \"name\": \"factory\"," >> $manifest
echo "                \"address\": \"$factory_offset\"," >> $manifest
echo "                \"file\": \"factory.bin\"" >> $manifest
echo "            }," >> $manifest
echo "            {" >> $manifest
echo "                \"name\": \"storage\"," >> $manifest
echo "                \"address\": \"$storage_offset\"," >> $manifest
echo "                \"file\": \"storage.bin\"," >> $manifest
echo "                \"isStorage\": true" >> $manifest
echo "            }" >> $manifest
echo "        ]" >> $manifest
echo "    }" >> $manifest
echo "}" >> $manifest

# generate package.tar.gz
(cd $tmp && tar -czf ../$board.tar.gz *)
rm -rf $tmp

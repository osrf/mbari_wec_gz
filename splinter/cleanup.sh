SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

pushd $SCRIPT_DIR
rm -rf assets/ CREDITS.md docs/ include/ lib README_SPLINTER.md splinter/
popd

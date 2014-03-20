#!/bin/bash

# Check that we were passed at least two arguments
if [ $# -lt 2 ]; then
	echo "$(basename $0) needs at least two arguments: template file and output file"
	exit 1
fi

# Check that the template file actually exists
if [ ! -f $1 ]; then
	echo "$1 cannot be used as a template because it does not exist"
	exit 2
fi

# If we were passed three or more arguments, assume the 3rd 
# argument is the directory in which to run git describe.
# Otherwise, we'll run git describe in template file directory
# (beause the template file should, in theory, be under version
# control).
if [ $# -ge 3 ]; then
	gitDir=$3
else
	gitDir=$(dirname ${1})
fi

# Check that the place in which we wish to run git describe is actually
# a directory.
if [ ! -d $gitDir ]; then
	echo "$gitDir is not a directory in which to run git describe"
	exit 3
fi

# Push the working directory onto the stack and change to the directory
# in which we've been instructed to run git describe. Run it, then pop
# the old working directory off the stack.  We redirect the results of 
# pushd and popd to /dev/null to suppress their output.
pushd . >> /dev/null
cd $gitDir
gitVersionStr=$(git describe --tags --long --dirty 2>/dev/null)
success=$?
popd >> /dev/null

# If git describe was successful, use sed to place the version description
# into the template file and write the output file.
if [ $success == 0 ]; then
	majorVersionStr=$(echo $gitVersionStr | sed -e 's/[^0-9]*\([0-9]*\).*/\1/')
	minorVersionStr=$(echo $gitVersionStr | sed -e 's/[^0-9]*[0-9]*\.\([0-9]*\).*/\1/')
	
	sed -e 's/\$GIT_VERSION_STR\$/'$gitVersionStr'/g' -e 's/\$MAJOR_VERSION_STR\$/'$majorVersionStr'/g' -e 's/\$MINOR_VERSION_STR\$/'$minorVersionStr'/g' < $1 > $2
else
	echo "Directory ${gitDir}/ is not a valid git repository"
	exit 4
fi


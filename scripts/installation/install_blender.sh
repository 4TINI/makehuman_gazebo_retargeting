#!/usr/bin/env bash

# https://download.blender.org/release/
echo "Installing Blender"
printf "\n"

version=3.2
subversion=2

releases_site=https://download.blender.org/release/

# Install Blender
echo "Downloading Blender's tarball..."
wget -c $releases_site/Blender$version/blender-$version.$subversion-linux-x64.tar.xz
printf "\n"
# if download failed
if ! [[ "$?" == 0 ]]
    then
        echo "The process of downloading tarball is failed!"
        printf "\n"
        exit
    else
        # extract & install
        echo "Extracting & Installing blender..."
        sudo tar -xf blender-$version.$subversion-linux-x64.tar.xz --one-top-level=blender-$version --strip-components 1 -C /opt
        
        sudo sed -i "s|Icon=blender|Icon=/opt/blender-$version/blender.svg|" /opt/blender-$version/blender.desktop
        rm -rf blender-$version.$subversion-linux-x64.tar.xz
        printf "\n"
fi

# make symlink
echo "Creating symlink..."
sudo ln -sv /opt/blender-$version/blender /usr/bin/blender
printf "\n"

# # check the symlink
echo "Checking symlink..."
ls -lAh /usr/bin/blender
printf "\n"

# # copying application entry
echo "Copying application entry..."
sudo cp -v /opt/blender-$version/blender.desktop /usr/share/applications
printf "\n"

# # Checking application entry
echo "Checking application entry"
ls -lAh /usr/share/applications/blender.desktop
printf "\n"

# # test the result of installation
echo "Checking blender installation..."
whereis blender
blender --version
printf "\n"

# # run
echo "Run blender..."
blender &
printf "\n"

echo "Done"
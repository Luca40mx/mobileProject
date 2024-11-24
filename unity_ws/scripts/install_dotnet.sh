#!/bin/bash
cd /workspace
curl -sSL https://dot.net/v1/dotnet-install.sh -o dotnet-install.sh
chmod u+x ./dotnet-install.sh
./dotnet-install.sh --channel 8.0
printf '\nexport DOTNET_ROOT=$HOME/.dotnet\n'  >> $HOME/.bashrc
printf '\nexport PATH=$PATH:$DOTNET_ROOT\n'  >> $HOME/.bashrc




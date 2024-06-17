#!/bin/bash
curl -L -o VirtualGym.zip --header "PRIVATE-TOKEN:glpat-MEz1Ab2kVz3Q9Umb9iVi" "https://gitlab.com/api/v4/projects/34020764/jobs/artifacts/develop/download?job=ros_generic_cell"
if [[ "$1" = "only_mesh" ]]
 then
  unzip -o VirtualGym.zip "build/acroba_resources/meshes/*" "build/acroba_resources/meshes/**/*" -d "./"
  unzip -o VirtualGym.zip "build/acroba_resources/models/*" "build/acroba_resources/models/**/*" -d "./"
  if [ -d "./acroba_resources/meshes" ]; then rm -Rf ./acroba_resources/meshes; fi
  if [ -d "./acroba_resources/models" ]; then rm -Rf ./acroba_resources/models; fi
  mv ./build/acroba_resources/meshes ./acroba_resources/meshes
  mv ./build/acroba_resources/models ./acroba_resources/models
  rm -r ./build
else
  curl -L -o VirtualGym_uc.zip --header "PRIVATE-TOKEN:glpat-MEz1Ab2kVz3Q9Umb9iVi" "https://gitlab.com/api/v4/projects/34020764/jobs/artifacts/develop/download?job=use_cases"
  find . -maxdepth 1 -type d -not -name 'VirtualGym*' -not -name '.' -not -name 'docs' -not -name 'README.md' -not -name '.git' -exec rm -rf {} \;
  unzip -o VirtualGym.zip -d .
  mv ./build/* ./ && mv ./build/.* ./ && rm -r ./build
  unzip -o VirtualGym_uc.zip -d .
  cp -r ./build/* ./ && rm -r ./build
  rm VirtualGym_uc.zip
  rm -rf ./virtual_scenes/*
  curl -o roscon_imr_cell.zip --header "PRIVATE-TOKEN:glpat-wrfmSxLFHh3KJy9awwAy" "https://gitlab.ti.bfh.ch/api/v4/projects/33485/repository/files/roscon_imr_cell.zip/raw?ref=main"
  unzip -o roscon_imr_cell.zip -d virtual_scenes
  chmod u+x ./virtual_scenes/roscon_imr_cell/roscon_imr_cell.x86_64
  rm roscon_imr_cell.zip
fi
rm VirtualGym.zip

model.avl and model.mass are the exact inputs used for the reported run.
commands.txt is its batch input; avl_output.txt contains ST, SB, eigenvalues and eigenvectors.
The airfoil and parameters are copies of inputs. Derivative units: angle per radian, rates per
nondimensional rate, and controls per degree. check_alpha.txt and check_elevator.txt record
independent fixed-control perturbations. The included script is the mode-analysis implementation;
the report run additionally requested the SB matrix. Run with AVL 3.52 in this folder:
  avl.exe < commands.txt
The executable is not included. Do not mix with older generated uam.avl files.

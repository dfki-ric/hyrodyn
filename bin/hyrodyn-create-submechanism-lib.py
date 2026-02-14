#!/usr/bin/env python

# mech_type_cap - mechanism type in capital letters for e.g. RRPR, SPU2U1
# mech_type_small - mechanism type (just capitalize the actuated joint) for e.g. rrPr, sPu2u1
# mechanism_geometry_schematic_filename - file name of the mechanism geometry schematic which will be used in the doxygen documentation
# ind_dof - independent dof of the mechanism
# st_dof - spanning tree dof of the mechanism
# lib_author_name - name of the contributing author
# lib_author_email - email of the contributing author
# include_in_hyrodyn - whether you want to include it in hyrodyn straightaway

print("Welcome to the HyRoDyn create submechanism library wizard!")
mech_type_small = input("Please name the type of the mechanism (e.g. rrPr, sPu2u1 etc): ")

mech_type_cap = mech_type_small.upper()

ind_dof = abs(int(input("Please enter the number of degrees of freedom (DOF) of the mechanism: ")))

st_dof = abs(int(input("Please enter the number of degrees of freedom (DOF) in its spanning tree: ")))

mechanism_geometry_schematic_filename = input("Please provide the file name of the mechanism schematic with its extension (e.g. lambda_mechansim.png) (optional, Press Enter to skip): ")

lib_author_name = input("Author Name: ")
if not lib_author_name:
    lib_author_name = "FirstName LastName"

lib_author_email = input("Author Email: ")
if not lib_author_email:
    lib_author_email = "FirstName.LastName@XYZ.com"

while 1:
    include_in_hyrodyn = input("Do you want to include it in hyrodyn right-away (Y/N)? Select No if you would like to have this library work with RBDL only. ")
    if include_in_hyrodyn not in ['y','Y','n','N']:
        print("Invalid Response! For yes, press y or Y and for no, press n or N.")
    else:
        break

with open("sub-mechanism_template/submechanism-lib-template.hpp", "rt") as fin:
    with open(mech_type_cap+".hpp", "wt") as fout:
        for line in fin:
            if include_in_hyrodyn in ['n','N']:
                fout.write(line.replace('mech_type_small', mech_type_small).replace('mech_type_cap',mech_type_cap).replace('ind_dof', str(ind_dof)).replace('st_dof', str(st_dof)).replace('lib_author_name', lib_author_name).replace('lib_author_email',lib_author_email).replace('mechanism_geometry_schematic_filename',mechanism_geometry_schematic_filename).replace('include_in_hyrodyn','//'))
            else:
                fout.write(line.replace('mech_type_small', mech_type_small).replace('mech_type_cap',mech_type_cap).replace('ind_dof', str(ind_dof)).replace('st_dof', str(st_dof)).replace('lib_author_name', lib_author_name).replace('lib_author_email',lib_author_email).replace('mechanism_geometry_schematic_filename',mechanism_geometry_schematic_filename).replace('include_in_hyrodyn',''))
    print(mech_type_cap + ".hpp" + " successfully created.")

with open("sub-mechanism_template/submechanism-lib-template.cpp", "rt") as fin:
    with open(mech_type_cap+".cpp", "wt") as fout:
        for line in fin:
            fout.write(line.replace('mech_type_small', mech_type_small).replace('mech_type_cap',mech_type_cap).replace('lib_author_name', lib_author_name).replace('lib_author_email',lib_author_email))
    print(mech_type_cap + ".cpp" + " successfully created.")

print("Submechanism library template for " + mech_type_small + " mechanism is ready. Please copy-paste your symbolic expressions of the loop closure functions inside the submechanism library and integrate it within the HyRoDyn software framework. Thanks for your contribution!")

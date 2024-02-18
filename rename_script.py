import os
result_folder = "./New Folder"
scenes_folders = os.listdir(result_folder)
for scene_folder in scenes_folders:
    dof_folders = os.listdir(os.path.join(result_folder,scene_folder))
    for dof in dof_folders:
        files = os.listdir(os.path.join(result_folder,scene_folder,dof,'a-star'))
        for file in files:
            if '.json' in file:
                os.rename(os.path.join(result_folder,scene_folder,dof,'a-star',file),os.path.join(result_folder,scene_folder,dof,'a-star','test_stats.json'))

        files = os.listdir(os.path.join(result_folder,scene_folder,dof,'rrt'))
        for file in files:
            if '.json' in file:
                os.rename(os.path.join(result_folder,scene_folder,dof,'rrt',file),os.path.join(result_folder,scene_folder,dof,'rrt','test_stats.json'))
       
import numpy as np
import cv2
from libs import *

import open3d as o3d
import copy
import sys

import os

voxel_size = 0.02
max_correspondence_distance_coarse = voxel_size * 15
max_correspondence_distance_fine = voxel_size * 1.5

def main(argv):
    

    poses = FileIO.getFromPickle(argv[1]+"/../../poses.pickle")
    print(poses)

    pcs= [None for x in range(len(poses['camnames']))]
    print(argv)



    for filename in os.listdir(argv[1]):
        if filename.endswith(".ply") or filename.endswith(".pcd"): 
            print(filename)

            filenn = filename.split(".")
            
            pcs[poses['camnames'].index(filenn[0])] = o3d.read_point_cloud(argv[1]+"/"+filename)
        else:
            continue

    


    visu.draw_geometry(pcs)
    for i in range(len(pcs)):
        H = mmnip.Rt2Homo(poses['R'][i],poses['t'][i].T)
        pcs[i].transform(H)



    visu.draw_geometry(pcs)
    quit()


    print("simple")
    pc = simpleMultiICP(pcs)
    visu.draw_geometry([pc])

    #pcres = MultiICPv2(pcs)
    #pcres = MultiICPv2(pcres)
    #pcres = MultiICPv2(pcres)



    


def MultiICPv2(pcs,voxel_radius=[0.04, 0.02, 0.01],max_iter=[50, 30, 14],lambda_geometric=1):
    #pcds_down = downsample(pcs,voxel_size)
    obs=[]

    for i in range(0,len(pcs)):
        for j in range(i+1,len(pcs)):   
            
            res_icp = colorICP(pcs[i],pcs[j],voxel_radius=voxel_radius,max_iter=max_iter,lambda_geometric=lambda_geometric)

            obs.append(obsgen.CreateObservation(i,j,'H',res_icp.transformation))


    for o in obs:
        wow = mmnip.Homo2Rt(o['H'])

        o['R']=wow[0]
        o['t']=wow[1]

    A = probdefs.rotationProbDef(obs,len(pcs))

    C = np.dot(A.T,A)

    Rs = algos.RProbSolv1(C,3,len(pcs))

    A,b= probdefs.translationProbDef(obs,Rs,len(pcs))

    #Solves Least Squres
    x= algos.LeastSquares(A,b)
    
    sol = np.split(x,len(pcs))

    #visu.ViewRefs(Rs,sol,showRef=True)

    
   

    for i in range(len(pcs)):
        pcs[i].transform(mmnip.Rt2Homo(Rs[i],np.squeeze(sol[i])))

    #visu.draw_geometry(pcs)

    return pcs


def simpleMultiICP(pcs,voxel_radius=[0.04],max_iter=[100],lambda_geometric=1):

    pccum = pcs[0]

    #stick them together 1 by 1
    for i in range(1,len(pcs)):

        resICP = colorICP(pccum,pcs[i],voxel_radius=voxel_radius,max_iter=max_iter,lambda_geometric=lambda_geometric)

        pccum.transform(resICP.transformation)

        pccum= pointclouder.MergeClouds([pccum,pcs[i]])



    return pccum


def downsample(pcs,voxel_si):

    pcds=[]
    for i in range(len(pcs)):
        pcd_down = o3d.geometry.voxel_down_sample(pcs[i], voxel_size=voxel_size)
        pcds.append(pcd_down)
    return pcds
    
#GAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA

def pairwise_registration(source, target):
    print("Apply point-to-plane ICP")

    o3d.geometry.voxel_down_sample(source,voxel_size=voxel_size)
    o3d.geometry.voxel_down_sample(target,voxel_size=voxel_size)
                                                       
    print("wow")
    #get normals to make ICP plane
    o3d.geometry.estimate_normals(source,o3d.geometry.KDTreeSearchParamHybrid(radius=max_correspondence_distance_fine * 2, max_nn=30))
    o3d.geometry.estimate_normals(target,o3d.geometry.KDTreeSearchParamHybrid(radius=max_correspondence_distance_fine * 2, max_nn=30))
    print("normals estimated")


    icp_coarse = o3d.registration.registration_icp(
        source, target, max_correspondence_distance_coarse, np.identity(4),
        o3d.registration.TransformationEstimationPointToPlane())
    icp_fine = o3d.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        icp_coarse.transformation,
        o3d.registration.TransformationEstimationPointToPlane())
    transformation_icp = icp_fine.transformation
    information_icp = o3d.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine,
        icp_fine.transformation)
    return transformation_icp, information_icp


def full_registration(pcds, max_correspondence_distance_coarse,
                      max_correspondence_distance_fine):
    pose_graph = o3d.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                pcds[source_id], pcds[target_id])
            print("Build o3d.registration.PoseGraph")
            if target_id == source_id + 1:  # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.registration.PoseGraphNode(np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.registration.PoseGraphEdge(source_id,
                                                   target_id,
                                                   transformation_icp,
                                                   information_icp,
                                                   uncertain=False))
            else:  # loop closure case
                pose_graph.edges.append(
                    o3d.registration.PoseGraphEdge(source_id,
                                                   target_id,
                                                   transformation_icp,
                                                   information_icp,
                                                   uncertain=True))
    return pose_graph

def ICPp2plane(pc1,pc2,voxel_radius=[0.04, 0.02, 0.01],max_iter=[50, 30, 14],current_transformation=np.identity(4)):

    source = pc1
    target = pc2

    
    for scale in range(len(voxel_radius)):
        
        print(max_iter)
        iter = max_iter[scale]
        radius = voxel_radius[scale]

        #downsample them according to specifications
        source_down = o3d.geometry.voxel_down_sample(source, radius)
        target_down = o3d.geometry.voxel_down_sample(target, radius)

        #get normals to make ICP plane
        o3d.geometry.estimate_normals(source_down,o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        o3d.geometry.estimate_normals(target_down,o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))

        
        result_icp = o3d.registration.registration_icp(
        source_down, target_down, radius, current_transformation,
        o3d.registration.TransformationEstimationPointToPlane(),o3d.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                    relative_rmse=1e-6,
                                                    max_iteration=iter))

                  
        current_transformation = result_icp.transformation
    
        #source.normals=o3d.Vector3dVector([])
        #target.normals=o3d.Vector3dVector([])

    return result_icp

def colorICP(pc1,pc2,voxel_radius=[0.04, 0.02, 0.01],max_iter=[50, 30, 14],current_transformation=np.identity(4),lambda_geometric=0.968):
    '''
    makes color iterative closest point

    lambda_geometric, importance given to geometry vs color, 1  makes this normal icp
    '''
    source = pc1
    target = pc2
    radius=-1
    
    for scale in range(len(voxel_radius)):
        

        iter = max_iter[scale]
        radius = voxel_radius[scale]

        #downsample them according to specifications
        source_down = o3d.geometry.voxel_down_sample(source, radius)
        target_down = o3d.geometry.voxel_down_sample(target, radius)

        #get normals to make ICP plane
        o3d.geometry.estimate_normals(source_down,o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))
        o3d.geometry.estimate_normals(target_down,o3d.geometry.KDTreeSearchParamHybrid(radius=radius * 2, max_nn=30))


        result_icp = o3d.registration.registration_colored_icp(source_down, target_down, radius, current_transformation,
            o3d.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                    relative_rmse=1e-6,
                                                    max_iteration=iter),lambda_geometric= lambda_geometric)
        current_transformation = result_icp.transformation


    #Fisher information matrix
    #information_icp = o3d.registration.get_information_matrix_from_point_clouds(source_down, target_down, radius,result_icp.transformation)


    return result_icp


if __name__ == '__main__':
    main(sys.argv)
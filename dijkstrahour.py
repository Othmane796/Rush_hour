# -*- coding: utf-8 -*-
"""
Created on Thu Nov 17 18:42:42 2016

@author: othmane
"""

import numpy as np
import time
import os
import subprocess
import sys, getopt

import matplotlib.pyplot as plt

def read_file(f):
	tab = []
	with open(f,'r') as fic:
		dim = list(map(int,fic.readline().split()))
		for lines in fic:
			x = lines.split()
			tab+=(x)

	return (tab,dim)

def get_data(platform,dim):
	"""
	@platform : An array representing the platform
	@dim : The dimensions of the platform [- , -]
	@return : A dictionary with the following format ("c1" : [position,size(2 ou 3),direction(0/h ou 1/v)]) ex "g" : [15,2,0]
	"""
	len_platform=dim[0]*dim[1]
	data={}
	if len(platform) != len_platform :
		raise Exception("Dimensions are wrong ...");

	for e in platform  :
		if e != '0' :
			if e in data:
				data[e][1]+=1
			else:
				data[e]=[-1,-1,-1]
				data[e][1]=1
				data[e][0]=platform.index(e)#returns the index of the first occurence of e in the list
				if platform[ data[e][0]+1 ] == e:
					data[e][2]=0
				elif platform[ data[e][0]+dim[1] ] == e :
					data[e][2]=1
				else:
					data[e][2]=-1 #Shouldn't happen, but just in case

	return data

def update_positions(configuration,d):
	"""
	@Configuration : A given state of the platform
	@d : The data array that contains the information about the current state of the  platform
	@return : nothing, updates the data array (d) using the information in Configuration
	"""
	for e in d:
		d[e]=[configuration.index(e),d[e][1],d[e][2]]

def config_voisins(configuration,dim,data):
	"""
	@Configuration : A given state of the platform for which we want to find neighbors, a neighbor is a state we can get to by moving a car
	@dim : The dimensions of the platform [- , -]
	@data : The information about the current state of the platform (cars and their positions ..Etc)
	@data : Is represented as follows : data["t1"]=[position,taille,direction]=[17,3,1] whre "t1" is the name of a vehicule
	@return : The neighbor of the state "configuration" and the number of moves we need to get there as a list [(neighbor,number of moves),...]
	"""
	update_positions(configuration,data)
	voisins=[]
	for e in data:
		if data[e][1] == 2:
			taille=2
		else:
			taille=3

		if data[e][2] == 0:
			posi=data[e][0]
			j=posi+taille
			while(j < posi+dim[1]-posi%dim[1]):#sort de la boucle si on depasse la platform
				if(configuration[j] == '0'):
					config_voisin=list(configuration)
					config_voisin[posi:posi+taille]=['0' for k in range(taille)] #vider les cases occupé
					config_voisin[j-taille+1:j+1]=[e for k in range(taille)] #remplir les cases apres le deplacement
					voisins.append([config_voisin,j-posi-taille+1])
					j+=1
				else:
					break

			j=posi-1
			while( j >=posi-posi%dim[1] ):
				if(configuration[j] == '0'):
					config_voisin=list(configuration)
					config_voisin[posi:posi+taille]=['0' for k in range(taille)] #vider les cases occupé
					config_voisin[j:j+taille]=[e for k in range(taille)] #remplir les cases apres le deplacement
					voisins.append([config_voisin,posi-j])

					j-=1
				else:
					break

		if data[e][2] == 1:
			posi=data[e][0]
			j=posi+(taille*dim[1])
			while(j < posi+(dim[0]-posi//dim[0])*dim[1] ) :#sort de la boucle si on depasse la platform
				if( configuration[j] == '0'):
					config_voisin=list(configuration)
					#vider les cases occupé
					#remplir les cases apres le deplacement
					for k in range(taille):
						config_voisin[posi+dim[1]*k]='0'

					for k in range(taille):
						config_voisin[j-dim[1]*k]=e



					voisins.append([config_voisin,j//dim[1]-posi//dim[1]-taille+1])

					j+=dim[1]
				else:
					break

			j=posi-dim[1]
			while(j >= posi-(posi//dim[0])*dim[1] ) :#sort de la boucle si on depasse la platform
				if( configuration[j] == '0'):
					config_voisin=list(configuration)
					#vider les cases occupé
					#remplir les cases apres le deplacement
					for k in range(taille):
						config_voisin[posi+dim[1]*k]='0'
						config_voisin[j+dim[1]*k]=e
					voisins.append([config_voisin,posi//dim[1]-j//dim[1]])
					j-=dim[1]
				else:
					break

	return voisins

def same_config(config1,config2):
	"""
	@config1 : A given state of the platform
	@config2 : A given state of the platform
	@return : True  if it's the same states, false otherwise
	"""
	if len(config1) != len(config2):
		raise Exception("length is not ok")
	else:
		for i in range(len(config1)):
			if config1[i] != config2[i]:
				return False
		return True

def solver(configuration_init,dim,data,RHM,total=False):
	"""
	@configuration_init : the initiale state of the platform
	@dim : The dimensions of the platform [- , -]
	@data : The information about the current state of the platform (cars and their positions ..Etc)
 	@RHM: True, solve RHM (Rush hour mouvement) , otherwise solves RHC (Rush hour cases)
	@total : True if we want to explore all nodes, otherwise false
	@return : (arcs,nodes,predecessor,distances,path)
		arcs is a dictionnary {index : state of platform}
		nodes is a 2D Array len(nodes) x len(nodes)
		predecessor is a dictionnary {index : predecessor of the node index}
		distances is a dictionnary {index : minimal distance of the node index }
		paht is a liste of indexs representing  the solution of the game
	"""
	s_fin=-1 #index of the finale node
	distances={0 : 0}
	predecessor={}
	sommets_o=dict() #contains the open nodes and their predecessors [index : predecessors]
	nodes={0 : configuration_init}
	arcs_tmp={0 : set()} #{index : set containing the successors of index}

	cmpt=1
	voisins=config_voisins(configuration_init,dim,data)
	sommets_o[0]=list()
	for e in voisins:
		sommets_o[cmpt]=[0]
		nodes[cmpt]=e[0]
		arcs_tmp[0].add((cmpt,e[1]))
		if RHM :
			distances[cmpt]=1
		else:
			distances[cmpt]=e[1]

		predecessor[cmpt]=0
		cmpt+=1

	sommets_o.pop(0)

	while len(sommets_o) != 0 :
		smin=find_min(sommets_o,distances)
		pred=sommets_o.pop(smin)
		index=smin
		if nodes[index][16:18] == ['g','g']:#as soon as we find a finale node as minimum we stop looking
			s_fin=index
			if total == False:
				break

		current_config=nodes[index]
		L=config_voisins(current_config,dim,data)
		arcs_tmp[index]=set()
		for i in range(len(L)):
			bool=0
			for p in pred:#checking that c is not a predecessor
				if same_config(L[i][0],nodes[p]):
					bool=1
					break

			if bool == 0:
				for e in sommets_o.keys():
					#update if c is an open node
					if same_config(L[i][0],nodes[e]):
						sommets_o[e].append(index)#we just found an open as a predecessor of index we update this information
						if RHM :
							arcs_tmp[index].add((e,1))#and here as a successor of index
							if distances[e] > distances[index]+1 :
								distances[e]=distances[index]+1
								predecessor[e]=index
						else:
							arcs_tmp[index].add((e,L[i][1]))##and here as a successor of index
							if distances[e] > distances[index]+L[i][1] :
								distances[e]=distances[index]+L[i][1]
								predecessor[e]=index


						bool=1
						break

			if bool == 0:
				#new neighbor
				sommets_o[cmpt]=[index]
				nodes[cmpt]=L[i][0]
				if RHM:
					arcs_tmp[index].add((cmpt,1))
					distances[cmpt]=distances[index]+1
				else:
					arcs_tmp[index].add((cmpt,L[i][1]))
					distances[cmpt]=distances[index]+L[i][1]
				predecessor[cmpt]=index
				cmpt+=1

	if s_fin != -1:

		#finds the path using s_fin
		path=[]
		s=s_fin
		s_init=0
		while s != s_init:
			path.append(s)
			s=predecessor[s]

		path.append(s)
		path.reverse()

		#we change the represenation of nodes and arcs from list of successors to a matrix
		arcs=np.zeros([len(nodes),len(nodes)])
		for n in arcs_tmp:
			voisins=arcs_tmp[n]
			for v in voisins:
				arcs[n][v[0]]=v[1]
				arcs[v[0]][n]=v[1]

		return (nodes,arcs,distances,predecessor,path)
	else:
		print("There is no solution \n")
		return

def print_tab_terminal(tab):
	"""
	@tab : A given state of the platform
	@return : nothing, prints the platform in 2D
	"""
	t = ['+---' for i in range(6)]
	strt = (''.join([str(x) for x in t])+"+")
	i = 0
	while(i<len(tab)//6):
		print(strt)
		#tab2=['|'+'{:3d}'.format(x) for x in tab[i]]
		l=[]
		for j in range(0,6):
			l.append('|'+'{:3s}'.format(tab[6*i+j]))
		l.append('|')
		print(''.join(l))
		i+=1
	print(strt)

def find_min(Q,distances):
	"""
	@Q : A dictinnary containing open nodes
	@distance : A dictionnary containing the minimale distances to reach a certain node
	@return : The index of the node with the smalest distance
	"""
	mini=float("inf")
	sommet=-1
	for e in Q:
		if distances[e] < mini:
			mini=distances[e]
			sommet=e

	return sommet
"""

def testing_jams():
	with open("test1.txt",'w') as f:
		f.write("Beginner puzzles\n")
		proc=subprocess.Popen(["ls ./puzzles/Beginner"], stdout=subprocess.PIPE, shell=True,universal_newlines=True)
		(out,err)=proc.communicate()
		puzzles=out.split("\n")
		for p in puzzles[:-1]:
			(tab,dim)=read_file("./puzzles/Beginner/"+p)
			data=get_data(tab,dim)
			start=time.clock()
			nodes,arcs,explorer,pred,A=solver(tab,dim,data,False,False)
			end=time.clock()
			spent=end-start
			print(p,spent)
			f.write(p.split('.')[0]+ "\t %f \n" %(spent))

		f.write("Intermediary puzzles\n")
		proc=subprocess.Popen(["ls ./puzzles/Intermediary"], stdout=subprocess.PIPE, shell=True,universal_newlines=True)
		(out,err)=proc.communicate()
		puzzles=out.split("\n")
		for p in puzzles[:-1]:
			(tab,dim)=read_file("./puzzles/Intermediary/"+p)
			data=get_data(tab,dim)
			start=time.clock()
			nodes,arcs,explorer,pred,A=solver(tab,dim,data,False,False)
			end=time.clock()
			spent=end-start
			print(p,spent)
			f.write(p.split('.')[0]+ "\t %f \n" %(spent))

		f.write("Advanced puzzles\n")
		proc=subprocess.Popen(["ls ./puzzles/Advanced/"], stdout=subprocess.PIPE, shell=True,universal_newlines=True)
		(out,err)=proc.communicate()
		puzzles=out.split("\n")
		for p in puzzles[:-1]:
			(tab,dim)=read_file("./puzzles/Advanced/"+p)
			data=get_data(tab,dim)
			start=time.clock()
			nodes,arcs,explorer,pred,A=solver(tab,dim,data,False,False)
			end=time.clock()
			spent=end-start
			print(p,spent)
			f.write(p.split('.')[0]+ "\t %f \n" %(spent))

		f.write("Expert puzzles\n")
		proc=subprocess.Popen(["ls ./puzzles/Expert"], stdout=subprocess.PIPE, shell=True,universal_newlines=True)
		(out,err)=proc.communicate()
		puzzles=out.split("\n")
		for p in puzzles[:-1]:
			(tab,dim)=read_file("./puzzles/Expert/"+p)
			data=get_data(tab,dim)
			start=time.clock()
			nodes,arcs,explorer,pred,A=solver(tab,dim,data,False,False)
			end=time.clock()
			spent=end-start
			print(p,spent)
			f.write(p.split('.')[0]+ "\t %f \n" %(spent))

def testing_jams_nodes():
	x_some=list()
	y_some=list()
	x_all=list()
	y_all=list()
	with open("test2.txt",'w') as f:
		f.write("Beginner puzzles\n")
		f.write("Jam \t some\t time\t all\t time\t \n")
		proc=subprocess.Popen(["ls ./puzzles/Beginner"], stdout=subprocess.PIPE, shell=True,universal_newlines=True)
		(out,err)=proc.communicate()
		puzzles=out.split("\n")
		for p in puzzles[:-1]:
			(tab,dim)=read_file("./puzzles/Beginner/"+p)
			data=get_data(tab,dim)
			start=time.clock()
			nodes,arcs,explorer,pred,A=solver(tab,dim,data,False,False)
			end=time.clock()
			spent_some=end-start

			start=time.clock()
			nodes_bis,arcs_bis,explorer_bis,pred_bis,A=solver(tab,dim,data,False,True)
			end=time.clock()
			spent_all=end-start
			x_some.append(len(nodes))
			y_some.append(spent_some)
			x_all.append(len(nodes_bis))
			y_all.append(spent_all)
			print(p,len(nodes),spent_some,len(nodes_bis),spent_all)
			f.write(p.split('.')[0]+ "\t %d\t %.2f\t %d\t %.2f\t \n" %(len(nodes),spent_some,len(nodes_bis),spent_all))

		f.write("Intermediary puzzles\n")
		f.write("Jam \t some\t time\t all\t time\t \n")
		proc=subprocess.Popen(["ls ./puzzles/Intermediary"], stdout=subprocess.PIPE, shell=True,universal_newlines=True)
		(out,err)=proc.communicate()
		puzzles=out.split("\n")
		for p in puzzles[:-1]:
			(tab,dim)=read_file("./puzzles/Intermediary/"+p)
			data=get_data(tab,dim)

			if p == "jam14.txt":
				start=time.clock()
				nodes,arcs,explorer,pred,A=solver(tab,dim,data,False,False)
				end=time.clock()
				spent_some=end-start
				print(p,len(nodes),spent_some,"too big",float('inf'))
				f.write(p.split('.')[0]+ "\t %d\t %.2f\t %f\t %f\t \n" %(len(nodes),spent_some,float('inf'),float('inf')))


			else:
				start=time.clock()
				nodes,arcs,explorer,pred,A=solver(tab,dim,data,False,False)
				end=time.clock()
				spent_some=end-start

				start=time.clock()
				nodes_bis,arcs_bis,explorer_bis,pred_bis,A=solver(tab,dim,data,False,True)
				end=time.clock()
				spent_all=end-start

				x_some.append(len(nodes))
				y_some.append(spent_some)
				x_all.append(len(nodes_bis))
				y_all.append(spent_all)
				print(p,len(nodes),spent_some,len(nodes_bis),spent_all)
				f.write(p.split('.')[0]+ "\t %d\t %.2f\t %d\t %.2f\t \n" %(len(nodes),spent_some,len(nodes_bis),spent_all))

		f.write("Advanced puzzles\n")
		f.write("Jam \t some\t time\t all\t time\t \n")
		proc=subprocess.Popen(["ls ./puzzles/Advanced/"], stdout=subprocess.PIPE, shell=True,universal_newlines=True)
		(out,err)=proc.communicate()
		puzzles=out.split("\n")
		for p in puzzles[:-1]:
			(tab,dim)=read_file("./puzzles/Advanced/"+p)
			data=get_data(tab,dim)
			start=time.clock()
			nodes,arcs,explorer,pred,A=solver(tab,dim,data,False,False)
			end=time.clock()
			spent_some=end-start

			start=time.clock()
			nodes_bis,arcs_bis,explorer_bis,pred_bis,A=solver(tab,dim,data,False,True)
			end=time.clock()
			spent_all=end-start

			x_some.append(len(nodes))
			y_some.append(spent_some)
			x_all.append(len(nodes_bis))
			y_all.append(spent_all)

			print(p,len(nodes),spent_some,len(nodes_bis),spent_all)
			f.write(p.split('.')[0]+ "\t %d\t %.2f\t %d\t %.2f\t \n" %(len(nodes),spent_some,len(nodes_bis),spent_all))

		f.write("Expert puzzles\n")
		f.write("Jam \t some\t time\t all\t time\t \n")
		proc=subprocess.Popen(["ls ./puzzles/Expert"], stdout=subprocess.PIPE, shell=True,universal_newlines=True)
		(out,err)=proc.communicate()
		puzzles=out.split("\n")
		for p in puzzles[:-1]:
			(tab,dim)=read_file("./puzzles/Expert/"+p)
			data=get_data(tab,dim)
			start=time.clock()
			nodes,arcs,explorer,pred,A=solver(tab,dim,data,False,False)
			end=time.clock()
			spent_some=end-start

			start=time.clock()
			nodes_bis,arcs_bis,explorer_bis,pred_bis,A=solver(tab,dim,data,False,True)
			end=time.clock()
			spent_all=end-start

			x_some.append(len(nodes))
			y_some.append(spent_some)
			x_all.append(len(nodes_bis))
			y_all.append(spent_all)
			print(p,len(nodes),spent_some,len(nodes_bis),spent_all)
			f.write(p.split('.')[0]+ "\t %d\t %.2f\t %d\t %.2f\t \n" %(len(nodes),spent_some,len(nodes_bis),spent_all))

	return x_some,y_some,x_all,y_all

def RHC_RHM_tests():
	proc=subprocess.Popen(["ls ./puzzles/Beginner"], stdout=subprocess.PIPE, shell=True,universal_newlines=True)
	(out,err)=proc.communicate()
	puzzles=out.split("\n")
	x_RHC=list()
	y_RHC=list()
	x_RHM=list()
	y_RHM=list()
	for p in puzzles[:-1]:
		(tab,dim)=read_file("./puzzles/Beginner/"+p)
		data=get_data(tab,dim)

		start=time.clock()
		nodes,arcs,explorer,pred,A=solver(tab,dim,data,False,True)
		end=time.clock()
		spent=end-start
		x_RHC.append(len(nodes))
		y_RHC.append(spent)

		start=time.clock()
		nodes,arcs,explorer,pred,A=solver(tab,dim,data,True,True)
		end=time.clock()
		spent=end-start
		x_RHM.append(len(nodes))
		y_RHM.append(spent)

	plt.plot(x_RHM,y_RHM,'ro',label="RHM")
	plt.plot(x_RHC,y_RHC,'bo',label="RHC")

	plt.title("Debutant puzzles : RHC vs RHM")
	plt.legend()

	plt.show()

def plot_tests():

	proc=subprocess.Popen(["ls ./puzzles/Expert"], stdout=subprocess.PIPE, shell=True,universal_newlines=True)
	(out,err)=proc.communicate()
	puzzles=out.split("\n")
	x=list()
	y=list()
	for p in puzzles[:-1]:
		(tab,dim)=read_file("./puzzles/Expert/"+p)
		data=get_data(tab,dim)

		start=time.clock()
		nodes,arcs,explorer,pred,A=solver(tab,dim,data,False,True)
		end=time.clock()
		spent_all=end-start
		x.append(len(nodes))
		y.append(spent_all)

	proc=subprocess.Popen(["ls ./puzzles/Advanced"], stdout=subprocess.PIPE, shell=True,universal_newlines=True)
	(out,err)=proc.communicate()
	puzzles=out.split("\n")

	for p in puzzles[:-1]:
		(tab,dim)=read_file("./puzzles/Advanced/"+p)
		data=get_data(tab,dim)

		start=time.clock()
		nodes,arcs,explorer,pred,A=solver(tab,dim,data,False,True)
		end=time.clock()
		spent_all=end-start
		x.append(len(nodes))
		y.append(spent_all)



	plt.plot(x,y,'ro')
"""


if __name__ == "__main__":
	try:
		opts, agrs = getopt.getopt(sys.argv[1:],"i:r:a:p:",["ifile=","RHC","All","printit"])
	except getopt.GetoptError as err:
		print(str(err))
		sys.exit()

	d_opts=dict(opts)
	print(d_opts)

	(tab,dim)=read_file(d_opts["-i"])
	data=get_data(tab,dim)
	if d_opts["-r"] == "RHM":

		if d_opts["-a"] == "All":
			nodes,arcs,explorer,pred,A=solver(tab,dim,data,True,True)
		elif d_opts["-a"] == "Some":
			nodes,arcs,explorer,pred,A=solver(tab,dim,data,True,False)
		else:
			raise Exception(" 'All' or 'Some' please nothing else \n" )
			sys.exit()

	elif d_opts["-r"] =="RHC":

		print("RHC")
		if d_opts["-a"] == "All":
			nodes,arcs,explorer,pred,A=solver(tab,dim,data,False,True)
		elif d_opts["-a"] == "Some":
			nodes,arcs,explorer,pred,A=solver(tab,dim,data,False,False)
		else:
			raise Exception(" 'All' or 'Some' please nothing else \n" )
			sys.exit()

	else:
		raise Exception(" 'RHC' or 'RHM' please nothing else \n")
		sys.exit()


	print(d_opts["-p"])

	if d_opts["-p"] =="Yes":
		print("The path reperesenting the solution is the following\n")
		print(A)
		for e in A:
			print_tab_terminal(nodes[e])
			print("///////////////////////////\n")
	elif d_opts["-p"] == "No":
		print("The path reperesenting the solution is the following\n")
		print(A)
	else:
		raise Exception("Y or N : Do you want to print the states that will get you to the solution ?\n")
		sys.exit()




"""
#All test functions are commented
RHC_RHM_tests()
plot_tests()
testing_jams()


x_some,y_some,x_all,y_all=testing_jams_nodes()

M=1
plt.plot(x_some[:-M],y_some[:-M],'ro',label="Some nodes")
plt.plot(x_all[:-M],y_all[:-M],'bo',label="All nodes")

plt.legend()

plt.show()

"""

//import java.awt.List;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Map.Entry;


/**
 * An algorithm that computes a solution of the motion planning problem. <br>
 * 
 * @author Luca Castelli Aleardi (INF421, Ecole Polytechnique, dec 2020)
 *
 */
public class MyBestAlgorithm extends MotionAlgorithm {
	/** An input instance of the motion planning problem */
	public int m = 0;
	public Instance input;
	int juju =0;
	
	/** The solution computed by the algorithm */
	public Solution solution;
	
	/** Current locations of robots */
	Coordinates current;
	/*Marge qui définit à quel point les robots peuvent sortir de la bounding box*/
	public static final int marge = 4;
	/*Liste contenant les indices de robots non encore arrivés à destination*/
	public ArrayList<Integer> indices;
	/*pour chaque robot, une matrice qui contient les distances de tous les points du plan au point d'arrivee du robot considere*/
	public ArrayList<HashMap<Integer, HashMap<Integer, Integer>>> listeDamier; 
	/*les couleurs utilisees par l'algorithme de BFS pour remplir listeDamier*/
	public ArrayList<HashMap<Integer, HashMap<Integer, String>>> listeCouleur; 
	/*Liste d'attente des robots qui bloquent la cible d'un autre robot*/
	public ArrayList<Integer> waitList = new ArrayList<Integer>();//celle ci est temporaire et se vide à la fin de chaque step
	public ArrayList<Integer> waitListFinal = new ArrayList<Integer>();
	//ArrayList<Node<Coordinates>> listeArbre = new ArrayList<Node<Coordinates>>();
	/**/
	HashMap<Integer, ArrayList<Coordinates>> startDest = new HashMap<Integer, ArrayList<Coordinates>>();//contient les positions actuelles et prochaines pour chaque robot
	HashMap<Integer, Integer> radiuses = new HashMap<Integer, Integer>();//dans le cas où l'on doit écarter des robots qui piègent des cibles, il contient la longueur des cotés des carrés concentriques d'ecartement
	
	HashMap<Integer, HashMap<Integer, Integer>> libre = new HashMap<Integer, HashMap<Integer, Integer>>();//position occupees des obstacles
	
	HashMap<Integer, HashMap<Integer, Integer>> positions = new HashMap<Integer, HashMap<Integer, Integer>>();//position occupees a tout instant t
	/*utile pour les initialisations*/
	HashMap<Integer, HashMap<Integer, Integer>> damier ;
	/*utile pour les initialisations*/
	HashMap<Integer, HashMap<Integer, String>> couleur ;
	/*Affiche les bornes inférieures pour le makespan et la distance totale (dans le cas idéal)*/
	public void affichage() {
		int n = current.n;
		int distance = 0;
		int maximum = 0, intermediaire;
		for(int i = 0; i < n; i++) {
			intermediaire = java.lang.Math.abs(input.starts.getX(i) - input.targets.getX(i)) + java.lang.Math.abs(input.starts.getY(i) - input.targets.getY(i));
			distance = distance + intermediaire;
			if(intermediaire > maximum) {maximum = intermediaire;}
		}
		System.out.println("La distance totale à vol d'oiseau est : "+distance);
		System.out.println("Borne inferieure triviale pour le Makespan : "+maximum);
	}
	public MyBestAlgorithm(Instance input) {
		this.input=input;
		this.solution=new Solution(input.name); // create an empty solution (no steps at the beginning)
		this.current=new Coordinates(this.input.starts.getPositions()); // initialize the current locations with the starting input locations
		listeDamier = new ArrayList<HashMap<Integer, HashMap<Integer, Integer>>>();
		listeCouleur = new ArrayList<HashMap<Integer, HashMap<Integer, String>>>();
	}
	
	/**
	 * Return the current solution: it assumes that the solution has been computed
	 */
	public Solution getSolution() {
		return this.solution;
	}
	/*Initialisation du tableau des positions*/
	public void init_positions() {
		//On élargit d'abord la bounding box en se servant de la marge
		int x_min = this.input.xmin - MyBestAlgorithm.marge;
		int x_max = this.input.xmax + MyBestAlgorithm.marge;
		int y_min = this.input.ymin - MyBestAlgorithm.marge;
		int y_max = this.input.ymax + MyBestAlgorithm.marge;
		for(int i = x_min; i<= x_max; i++) {
			//Ajout des lignes
			libre.put(i, new HashMap<Integer, Integer>()); 
			positions.put(i, new HashMap<Integer, Integer>());
			for(int j = y_min; j<= y_max; j++) {
				//Ajout des colonnes
				libre.get(i).put(j, 0); 
				positions.get(i).put(j, 0);
			}
		}
		/*
		 *Ensuite on met les cases correspond aux obstacles à 1 dans les tableaux libre et positions 
		 *pour signifier que ces cases sont occupées.
		 * */
		int n = this.input.obstacles.size();
		int x, y;
		for(int k = 0; k < n; k++) {
			x = this.input.obstacles.getX(k);
			y = this.input.obstacles.getY(k);
			libre.get(x).put(y, 1);
			positions.get(x).put(y, 1);
		}
		for(int k = 0; k < this.current.n; k++) {
			x = this.current.getX(k);
			y = this.current.getY(k);
			positions.get(x).put(y, 1);
			radiuses.put(k, 0);
		}
	}
	
	
	/**
	 * Transforme un couple d'entiers en un type Coordinates. Le premier param
	 * */
	public Coordinates create_coordinate(int i, int j) {
		int[][] array = new int[2][1];
		array[0][0] = i;
		array[1][0] = j;
		return new Coordinates(array);
	}
	
	
	/*
	 * Initialise les variables damier et couleur qui seront utilisées pour le BFS
	 * */
	public void init() {
		int x_min = this.input.xmin - MyBestAlgorithm.marge;
		int x_max = this.input.xmax + MyBestAlgorithm.marge;
		int y_min = this.input.ymin - MyBestAlgorithm.marge;
		int y_max = this.input.ymax + MyBestAlgorithm.marge;

		damier = new HashMap<Integer, HashMap<Integer, Integer>>();
		couleur = new HashMap<Integer, HashMap<Integer, String>>();
		for(int i = x_min; i <= x_max ; i++) {
			damier.put(i, new HashMap<Integer, Integer>());
			couleur.put(i, new HashMap<Integer, String>());
			for(int j = y_min; j <= y_max; j++) {
				damier.get(i).put(j, (int)Float.POSITIVE_INFINITY);
				couleur.get(i).put(j, "BLANC");
			}
		}
	}
	/**
	 * Même travail que la fonction init() mais uniquement pour les robots
	 * n'ayant pas encore atteint leur cible
	 */
	public void reInit() {
		int x_min = this.input.xmin - MyBestAlgorithm.marge;
		int x_max = this.input.xmax + MyBestAlgorithm.marge;
		int y_min = this.input.ymin - MyBestAlgorithm.marge;
		int y_max = this.input.ymax + MyBestAlgorithm.marge;

		for(int k = 0; k<this.input.n; k++) {
			if(input.targets.getX(k) != current.getX(k) || input.targets.getY(k)!= current.getY(k)) {
				for(int i = x_min; i <= x_max ; i++) {
					for(int j = y_min; j <= y_max; j++) {
							listeDamier.get(k).get(i).put(j, (int)Float.POSITIVE_INFINITY);
							listeCouleur.get(k).get(i).put(j, "BLANC");
						
					}
				}
			}
			
		}
		
	}
	/**
	 * Même travail que la fonction init() mais pour un seul robot
	 */
	public void reInitRobot(int k) {
		int x_min = this.input.xmin - MyBestAlgorithm.marge;
		int x_max = this.input.xmax + MyBestAlgorithm.marge;
		int y_min = this.input.ymin - MyBestAlgorithm.marge;
		int y_max = this.input.ymax + MyBestAlgorithm.marge;
		
			for(int i = x_min; i <= x_max ; i++) {
				for(int j = y_min; j <= y_max; j++) {
						listeDamier.get(k).get(i).put(j, (int)Float.POSITIVE_INFINITY);
						listeCouleur.get(k).get(i).put(j, "BLANC");
					
				}
			}
		
	}
	
	
	/**
	 * Renvoie les voisins libres d'un robot situé sur la case (i, j)
	 */	
	public Coordinates voisin(int i, int j) {
		int[][] array;
		int x_min = this.input.xmin - MyBestAlgorithm.marge;
		int x_max = this.input.xmax + MyBestAlgorithm.marge;
		int y_min = this.input.ymin - MyBestAlgorithm.marge;
		int y_max = this.input.ymax + MyBestAlgorithm.marge;
		if(i == x_min) {
			//On teste d'abord les cas particuliers des cases (x_min, y) qui ont soit deux voisins soit trois 
			if(j == y_min) {
				array = new int[2][2];
				array[0][0] = i + 1;
				array[1][0] = j;
				array[0][1] = i;
				array[1][1] = j + 1;
			}
			else if (j == y_max) {
				array = new int[2][2];
				array[0][0] = i + 1;
				array[1][0] = j;
				array[0][1] = i;
				array[1][1] = j - 1;
			}
			else {
				array = new int[2][3];
				array[0][0] = i + 1 ;
				array[1][0] = j;
				array[0][1] = i;
				array[1][1] = j + 1;
				array[0][2] = i;
				array[1][2] = j - 1;
			}
		}
		else if(i == x_max) {
			//Puis on teste les cas particuliers des cases (x_max, y) qui ont soit deux voisins soit trois 
			if(j == y_min) {
				array = new int[2][2];
				array[0][0] = i;
				array[1][0] = j + 1;
				array[0][1] = i - 1;
				array[1][1] = j;
			}
			else if (j == y_max) {
				array = new int[2][2];
				array[0][0] = i - 1;
				array[1][0] = j;
				array[0][1] = i;
				array[1][1] = j - 1;
			}
			else {
				array = new int[2][3];
				array[0][0] = i;
				array[1][0] = j + 1;
				array[0][1] = i - 1;
				array[1][1] = j;
				array[0][2] = i;
				array[1][2] = j - 1;
			}
		}
		else {
			//on teste le reste des cases (x, y_min), (x, y_max) et les cases hors bords qui ont 4 voisins
			if(j == y_min) {
				array = new int[2][3];
				array[0][0] = i + 1;
				array[1][0] = j;
				array[0][1] = i - 1;
				array[1][1] = j;
				array[0][2] = i;
				array[1][2] = j + 1;
			}
			else if (j == y_max) {
				array = new int[2][3];
				array[0][0] = i + 1;
				array[1][0] = j;
				array[0][1] = i - 1;
				array[1][1] = j;
				array[0][2] = i;
				array[1][2] = j - 1;
			}
			else {
				array = new int[2][4];
				array[0][0] = i;
				array[1][0] = j + 1;
				array[0][1] = i;
				array[1][1] = j - 1;
				array[0][2] = i + 1;
				array[1][2] = j;
				array[0][3] = i - 1;
				array[1][3] = j;
			}
		}
		//On vérifie ensuite lesquels de ces voisins sont libres
		int x, y, longueur = 0;
		int n = array[0].length;
		boolean[] bool = new boolean[n];
		for(int p = 0; p < n; p++) {
			x = array[0][p];
			y = array[1][p];
			
			if(libre.get(x).get(y) == 1 ) {
				bool[p] = false;
			}else {bool[p] = true;longueur++;}
		}
		int[][] tarray = new int[2][longueur];
		int f = 0;
		for(int p = 0; p < n; p++) {
			if(bool[p]) {
				tarray[0][f] = array[0][p];
				tarray[1][f] = array[1][p];
				f++;
			}
		}
		return new Coordinates(tarray);
	}
	
	
	/*
	 * Initialisation de listeDamier grâce au BFS (Breadth First Search)
	 * */
	public void initialisation() {
		Coordinates u, v;
		int n = this.input.n;
		int x, y, x_i, y_i, longueur;
		for(int k = 0; k < n; k++) {
			this.init();
			x = this.input.targets.getX(k);
			y = this.input.targets.getY(k);
			this.couleur.get(x).put(y, "GRIS");
			this.damier.get(x).put(y, 0);
			ArrayList<Coordinates> fifo = new ArrayList<Coordinates>();
			fifo.add(this.create_coordinate(x, y));
			while(!fifo.isEmpty()) {
				u = new Coordinates(fifo.get(0).getPositions());
				fifo.remove(0);
				v = voisin(u.getX(0), u.getY(0));
				longueur = v.size();
				for(int p = 0; p < longueur; p ++) {
					x_i = v.getX(p);
					y_i = v.getY(p);
					if(this.couleur.get(x_i).get(y_i).equals("BLANC")) {
						
						this.couleur.get(x_i).put(y_i, "GRIS");
						this.damier.get(x_i).put(y_i, 1 + this.damier.get(u.getX(0)).get(u.getY(0)));
						fifo.add(this.create_coordinate(x_i, y_i));
					}
				}
				this.couleur.get(u.getX(0)).put(u.getY(0), "NOIR");
			}
			HashMap<Integer, HashMap<Integer, Integer>> tdamier = new HashMap<Integer, HashMap<Integer, Integer>>(damier) ;
			HashMap<Integer, HashMap<Integer, String>> tcouleur = new HashMap<Integer, HashMap<Integer, String>>(couleur);
			listeDamier.add(k, tdamier);
			listeCouleur.add(k, tcouleur);
			//init_liste(k);
			this.damier.clear();
			this.couleur.clear();
			fifo.clear();
			}
			
		
	}
	
	
	/*
	 * Même travail que initialisation() mais pour les robots n'ayant pas encore atteints leur cible
	 * */
	public void reInitialisation() {
		reInit();
		Coordinates u, v;
		int x, y, x_i, y_i, longueur;
		
		for(int robot = 0; robot<this.input.n; robot++) {
			if(input.targets.getX(robot) != current.getX(robot) || input.targets.getY(robot)!= current.getY(robot)) {
				x = this.input.targets.getX(robot);
				y = this.input.targets.getY(robot);
				listeCouleur.get(robot).get(x).put(y, "GRIS");
				listeDamier.get(robot).get(x).put(y, 0);
				ArrayList<Coordinates> fifo = new ArrayList<Coordinates>();
				fifo.add(this.create_coordinate(x, y));
				while(!fifo.isEmpty()) {
					u = new Coordinates(fifo.get(0).getPositions());
					fifo.remove(0);
					v = voisin(u.getX(0), u.getY(0));
					longueur = v.size();
					
					for(int p = 0; p < longueur; p ++) {
						x_i = v.getX(p);
						y_i = v.getY(p);
						
						if(listeCouleur.get(robot).get(x_i).get(y_i).equals("BLANC")) {
							listeCouleur.get(robot).get(x_i).put(y_i, "GRIS");
							listeDamier.get(robot).get(x_i).put(y_i, 1 + listeDamier.get(robot).get(u.getX(0)).get(u.getY(0)));
							fifo.add(this.create_coordinate(x_i, y_i));
						}
						
					}
					listeCouleur.get(robot).get(u.getX(0)).put(u.getY(0), "NOIR");
				}
			}
		}
		
	}
	
	
	/*
	 * Même travail que initialisation() mais pour un seul robot
	 * */
	public void reInitialisationRobot(int k) {
		reInitRobot(k);
		Coordinates u, v;
		int x, y, x_i, y_i, longueur;
			
		x = this.input.targets.getX(k);
		y = this.input.targets.getY(k);
		listeCouleur.get(k).get(x).put(y, "GRIS");
		listeDamier.get(k).get(x).put(y, 0);
		ArrayList<Coordinates> fifo = new ArrayList<Coordinates>();
		fifo.add(this.create_coordinate(x, y));
		while(!fifo.isEmpty()) {
			u = new Coordinates(fifo.get(0).getPositions());
			fifo.remove(0);
			v = voisin(u.getX(0), u.getY(0));
			longueur = v.size();
			
			for(int p = 0; p < longueur; p ++) {
				x_i = v.getX(p);
				y_i = v.getY(p);
				
				if(listeCouleur.get(k).get(x_i).get(y_i).equals("BLANC")) {
					listeCouleur.get(k).get(x_i).put(y_i, "GRIS");
					listeDamier.get(k).get(x_i).put(y_i, 1 + listeDamier.get(k).get(u.getX(0)).get(u.getY(0)));
					fifo.add(this.create_coordinate(x_i, y_i));
				}
				
			}
			listeCouleur.get(k).get(u.getX(0)).put(u.getY(0), "NOIR");
		}
			
		
		
	}
	
	/*
	 * Définit le déplacement de chaque robot pour une itération
	 * Prend en paramètre la position (p, q) actuelle du robot, les coordonnées de ses voisins libres
	 * et l'indice du robot.
	 * */
	public byte mouvement(int p, int q, Coordinates coor, int robot) {
		int n = coor.n, x, y, longueur = 0;
		int x_etoile = Integer.MAX_VALUE, y_etoile = Integer.MAX_VALUE; //x_etoile et y_etoile sont les coordonnées de la potentielle prochaine case du robot
		if(n == 0) {
			//Si aucun voisin libre, le robot reste sur place le robot est piégé
			//System.out.println(current.getX(126)+","+current.getY(126));
			return Solution.FIXED;
		}else {
			int min = (int) Float.MAX_VALUE;
			for(int k = 0; k < n; k++) {
				x = coor.getX(k);
				y = coor.getY(k);
				if(positions.get(x).get(y) == 0) {
					longueur++;		//On vérifie que robot admet bien des voisins qui ne sont pas pris par d'autres robots
				}
			}
			if(longueur == 0) {return Solution.FIXED;}
			else {
				for(int k = 0; k < n; k++) {
					x = coor.getX(k);
					y = coor.getY(k);
					// Recherche de la case voisine ayant la plus petite distance et n'étant pas occupée par un des autres robots
					if((min > listeDamier.get(robot).get(x).get(y)) && positions.get(x).get(y) == 0 ) {
						min = listeDamier.get(robot).get(x).get(y);
						x_etoile = x;
						y_etoile = y;
					}
										
				}
				
				/*
				 * Cas où on n'a pas trouvée de voisin (c'est-à-dire x_étoile et y_etoile n'ont pas été actualisés
				 * */
				if(x_etoile == Integer.MAX_VALUE || y_etoile == Integer.MAX_VALUE) {
					reInitialisation();
					/*Boolean valant vrai si la cible du robot est piégé entre des robots ayant atteints leurs cibles*/
					boolean allPositionsMax = true;
					for(int i=0; i<coor.n; i++) {
						if(listeDamier.get(robot).get(coor.getX(i)).get(coor.getY(i))!= (int)Float.MAX_VALUE) {
							allPositionsMax = false;
							radiuses.put(robot, 0);
							
						}
					}
					
					if(allPositionsMax) {
						//Si la cible du robot est piégée on déplace les voisins de cette cible pour libérer la voie pour robot
						
						boolean yes = false;
						for(int k = 0; k<input.n; k++) {
							if((Math.abs(input.targets.getX(robot) - current.getX(k))<=radiuses.get(robot))
								&&(Math.abs(input.targets.getY(robot) - current.getY(k))<=radiuses.get(robot))) {
								
								Coordinates voisinCibles = voisin(current.getX(k), current.getY(k));
								if(voisinCibles.n != 0) {
									yes = true;
									int l;
									for(l = 0; l< voisinCibles.n; l++) {
										if((voisinCibles.getX(l)!=input.targets.getX(robot) || voisinCibles.getY(l)!=input.targets.getY(robot))) {
											/* Dans waitList on met les robots qui entourent la cible piégée, et on 
											 * les écartera pour libérer la voie pour robot
											 * */
											if(!waitList.contains(Integer.valueOf(k))){waitList.add(k);}
											if(!waitListFinal.contains(Integer.valueOf(k))){waitListFinal.add(k);indices.remove(Integer.valueOf(k));}
											break;
										}
									}
									
								}
							}
							if(!yes) {
								radiuses.put(robot, radiuses.get(robot) + 1);
								
								
							}
						}
					}

					return Solution.FIXED;
				}
				/*
				 * Cas où on a bien trouvé un voisin libre et non occupé, on se déplace vers ce voisin
				 * et on force la valeur infini pour la case precédente afin d'éviter les retours en arrière
				 * */
				listeDamier.get(robot).get(p).put(q, (int)Float.MAX_VALUE);
				return choisirDeplacement(x_etoile, y_etoile, p, q);		
			}
		}
	}
	/*
	 * Choisit le mouvement en fonction de la prochaine destination (x_etoile, y_etoile) du robot 
	 * et de sa position actuelle (p, q)
	 * */
	
	public byte choisirDeplacement(int x_etoile, int y_etoile, int p, int q) {
		if(x_etoile > p) {
			return Solution.E;
		}
		else if(x_etoile < p) {
			
			return Solution.W;
		}
		else {
			if(y_etoile > q) {
				return Solution.N;
			}else if (y_etoile < q){
				return Solution.S;
			}
			else {
				return Solution.FIXED;
			}
		}	
		
	}
		
	/**
	 * Compute a complete solution to the input problem: compute all steps, until all robots reach their target destinations
	 */
	public void run() {
		// TO BE COMPLETED
		this.init_positions();
		this.initialisation();
		this.indices = new ArrayList<Integer>();
		for(int p = 0; p < this.input.n; p++) {
			this.indices.add(p, Integer.valueOf(p));
		}
		//Tant que les listes indices ou waitList ne sont pas vides on cherche les deplacements des robots 
		while(!indices.isEmpty() || !waitListFinal.isEmpty()){
			if(indices.isEmpty()) {
				/*
				 * Si tous les robots dans indices sont arrivées à destination, on doit replacer les robots de waitList correctement
				 * On copie donc waitList dans indices et on relance l'algorithme
				 * */
				indices.addAll(waitListFinal);
				for(int i = 0; i<waitListFinal.size(); i++) {
					reInitialisationRobot(waitListFinal.get(i));
				}
				waitListFinal.clear();
			}
			computeOneStep();
		}
		System.out.println("Solution computed");
		this.affichage();
	}
	
	
	/**
	 * Add a new motion step to the current solution
	 */
	public void computeOneStep() {
		Coordinates intermediaire;
		int x, y, n = this.input.n;
		int robot, iterateur = 0;
		byte[] iteration = new byte[n];
		byte mouv;
		
		for(int p = 0; p < n; p++) {
			iteration[p] = Solution.FIXED;
			//On déplace les éléments de la liste d'attente de sorte à libérer la voie pour les robots restants
			if(!waitList.isEmpty()) {
				if(waitList.contains(Integer.valueOf(p))) {
					//System.out.println(p);
					Coordinates voisinsTemp = voisin(current.getX(p), current.getY(p));
					for(int l=0; l<voisinsTemp.n; l++) {
						int t=0;
						while(t<indices.size() && (voisinsTemp.getX(l)!=input.targets.getX(indices.get(t)) || voisinsTemp.getY(l)!=input.targets.getY(indices.get(t)))) {
							t++;
						}
						if(t>=indices.size()
								&& (positions.get(voisinsTemp.getX(l)).get(voisinsTemp.getY(l))==0)&& (listeDamier.get(p).get(voisinsTemp.getX(l)).get(voisinsTemp.getY(l))!=-1)) {
							
							iteration[p] = choisirDeplacement(voisinsTemp.getX(l), voisinsTemp.getY(l), current.getX(p), current.getY(p));
							
							int x1 = current.getX(p), y1 = current.getY(p);
							this.libre.get(x1).put(y1, 0);
							listeDamier.get(p).get(x1).put(y1, -1);
							switch(iteration[p]) {
								
								case Solution.N:
									this.positions.get(x1).put(y1, 0);
									this.positions.get(x1).put(y1+1, 1);
									this.libre.get(x1).put(y1+1, 1);
									current.increaseY(p);
									break;
								case Solution.S:
									this.positions.get(x1).put(y1, 0);
									this.positions.get(x1).put(y1-1, 1);
									this.libre.get(x1).put(y1-1, 1);
									current.decreaseY(p);
									break;
								case Solution.E:
									this.positions.get(x1).put(y1, 0);
									this.positions.get(x1+1).put(y1, 1);
									this.libre.get(x1+1).put(y1, 1);
									current.increaseX(p);
									break;
								case Solution.W:
									this.positions.get(x1).put(y1, 0);
									this.positions.get(x1-1).put(y1, 1);
									this.libre.get(x1-1).put(y1, 1);
									current.decreaseX(p);
									break;
								case Solution.FIXED:
									this.libre.get(x1).put(y1, 1);
									break;
							}
							break;
						}
					}
				}
				
			}	
			
		}
		waitList.clear();
		
		iterateur = 0;
		while(iterateur < this.indices.size()) {
			robot = this.indices.get(iterateur);
			x = this.current.getX(robot);
			y = this.current.getY(robot);
			if((x == this.input.targets.getX(robot))&&(y == this.input.targets.getY(robot))) {
				/*
				 * Si le robot "robot" est arrivé à destination on le retire de indices, on actualise libre de sorte à le considérer
				 * comme un obstacle, et on reinitialise de sorte à prendre en compte le nouvel obstacle. 
				 * */
				indices.remove(Integer.valueOf(robot));
				libre.get(x).put(y, 1);
				positions.get(x).put(y, 1);
				reInitialisation();
			}else {
				//on choisit le mouvement du robot puis on actualise le tableau de steps
				intermediaire = voisin(x, y);
				mouv = mouvement(x, y, intermediaire, robot);
				startDest.put(robot, new ArrayList<Coordinates>());
				startDest.get(robot).add(create_coordinate(x, y));
				switch(mouv) {
					case Solution.N:
						startDest.get(robot).add(create_coordinate(x, y+1));
						//current.increaseY(robot);
						break;
					case Solution.S:
						startDest.get(robot).add(create_coordinate(x, y-1));
						//current.decreaseY(robot);
						break;
					case Solution.E:
						startDest.get(robot).add(create_coordinate(x+1, y));
						//current.increaseX(robot);
						break;
					case Solution.W:
						startDest.get(robot).add(create_coordinate(x-1, y));
						//current.decreaseX(robot);
						break;
					case Solution.FIXED:
						startDest.get(robot).add(create_coordinate(x, y));
						break;
				}
				iteration[robot] = mouv;
				iterateur = iterateur + 1;
			}
			
			
		}
	
//		if(indices.size()==1 && (indices.get(0)==35||indices.get(0)==3)) {
//			juju++;
//			if(juju>=100) indices.clear();
//		}
		System.out.println("Robots restants: "+indices);
		
		iteration = checkCollisions(startDest, iteration);
		deplacement(iteration);
		this.solution.addStep(iteration);
		startDest.clear();
		//throw new Error("TO BE COMPLETED");
	}
	
	private byte[] checkCollisions(HashMap<Integer, ArrayList<Coordinates>> startDest, byte[] iteration) {
		// TODO Auto-generated method stub
		Iterator<Entry<Integer, ArrayList<Coordinates>>> hmIterator = startDest.entrySet().iterator();
		int i;
		while(hmIterator.hasNext()) {
			Map.Entry<Integer, ArrayList<Coordinates>> mapElement = (Map.Entry<Integer, ArrayList<Coordinates>>)hmIterator.next();
			i = mapElement.getKey();
			Coordinates voisin_i = voisin(startDest.get(i).get(0).getX(0), startDest.get(i).get (0).getY(0));
			if (voisin_i.n != 0) {
				for(int j =0; j<voisin_i.n; j++) {
					for(int robot: startDest.keySet()) {
						
						int a = this.current.getX(robot);
						int b = this.current.getY(robot);
						if(a==voisin_i.getX(j) && b==voisin_i.getY(j)) {
							//on verifie qu'il n'y ait pas de rotation
							if((mapElement.getValue().get(1).getX(0)== a && mapElement.getValue().get(1).getY(0) == b && scalarProduct(mapElement.getValue(), startDest.get(robot)) == 0)) {
								iteration[i] = Solution.FIXED;
								startDest.get(i).get(1).setX(0, startDest.get(i).get(0).getX(0) );
								startDest.get(i).get(1).setY(0, startDest.get(i).get(0).getY(0) );
							}
							else if((startDest.get(robot).get(1).getX(0)== mapElement.getValue().get(0).getX(0) && startDest.get(robot).get(1).getY(0) == mapElement.getValue().get(0).getY(0) && scalarProduct(mapElement.getValue(), startDest.get(robot)) == 0)) {
								iteration[robot] = Solution.FIXED;
								startDest.get(robot).get(1).setX(0, startDest.get(robot).get(0).getX(0) );
								startDest.get(robot).get(1).setY(0, startDest.get(robot).get(0).getY(0) );
							}
						}
					}
				}
			}
			//on vérifie qu'il n'y ait pas deux robots qui veulent occuper la même case
			for(int robot: startDest.keySet()) {
				if(robot!=i && (startDest.get(robot).get(1).getX(0)==startDest.get(i).get(1).getX(0) && startDest.get(robot).get(1).getY(0)==startDest.get(i).get(1).getY(0))) {
					iteration[i] = Solution.FIXED;
					startDest.get(i).get(1).setX(0, startDest.get(i).get(0).getX(0) );
					startDest.get(i).get(1).setY(0, startDest.get(i).get(0).getY(0) );
				}
			}
			
		}	
		
		return iteration;
	}
	private int scalarProduct(ArrayList<Coordinates> u, ArrayList<Coordinates> v) {
		
		return (u.get(1).getX(0) - u.get(0).getX(0))*(v.get(1).getX(0) - v.get(0).getX(0)) + (u.get(1).getY(0) - u.get(0).getY(0))*(v.get(1).getY(0) - v.get(0).getY(0)) ;
	}
	
	private void deplacement(byte[] iteration) {
		for(int i=0; i<indices.size(); i++) {
			int x = this.current.getX(indices.get(i));
			int y = this.current.getY(indices.get(i));
			switch(iteration[indices.get(i)]) {
			case Solution.N:
				this.positions.get(x).put(y, 0);
				this.positions.get(x).put(y+1, 1);
				current.increaseY(indices.get(i));
				break;
			case Solution.S:
				this.positions.get(x).put(y, 0);
				this.positions.get(x).put(y-1, 1);
				current.decreaseY(indices.get(i));
				break;
			case Solution.E:
				this.positions.get(x).put(y, 0);
				this.positions.get(x+1).put(y, 1);
				current.increaseX(indices.get(i));
				break;
			case Solution.W:
				this.positions.get(x).put(y, 0);
				this.positions.get(x-1).put(y, 1);
				current.decreaseX(indices.get(i));
				break;
			case Solution.FIXED:
				break;
			}
		}
	}

}

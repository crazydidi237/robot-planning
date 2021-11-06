//import java.awt.List;
import java.util.ArrayList;
import java.util.HashMap;


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
	
	/** The solution computed by the algorithm */
	public Solution solution;
	
	/** Current locations of robots */
	Coordinates current;
	/**/
	public static final int marge = 9;
	/**/
	public ArrayList<Integer> indices;
	/**/
	public ArrayList<HashMap<Integer, HashMap<Integer, Integer>>> listeDamier; //= new ArrayList<HashMap<Integer, HashMap<Integer, Integer>>>() ;
	/**/
	public ArrayList<HashMap<Integer, HashMap<Integer, String>>> listeCouleur; //= new ArrayList<HashMap<Integer, HashMap<Integer, String>>>();
	/**/
	//ArrayList<Node<Coordinates>> listeArbre = new ArrayList<Node<Coordinates>>();
	/**/
	HashMap<Integer, HashMap<Integer, Integer>> libre = new HashMap<Integer, HashMap<Integer, Integer>>();//position occupees des robots
	
	HashMap<Integer, HashMap<Integer, Integer>> positions = new HashMap<Integer, HashMap<Integer, Integer>>();//position occupees a tout instant t
	
	HashMap<Integer, HashMap<Integer, Integer>> damier ;
	HashMap<Integer, HashMap<Integer, String>> couleur ;
	
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
	//ma fonction/
	public void init_positions() {
		int x_min = this.input.xmin - MyBestAlgorithm.marge;
		int x_max = this.input.xmax + MyBestAlgorithm.marge;
		int y_min = this.input.ymin - MyBestAlgorithm.marge;
		int y_max = this.input.ymax + MyBestAlgorithm.marge;
		for(int i = x_min; i<= x_max; i++) {
			libre.put(i, new HashMap<Integer, Integer>());
			positions.put(i, new HashMap<Integer, Integer>());
			for(int j = y_min; j<= y_max; j++) {
				libre.get(i).put(j, 0);
				positions.get(i).put(j, 0);
			}
		}
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
		}
	}
	
	public Coordinates create_coordinate(int i, int j) {
		int[][] array = new int[2][1];
		array[0][0] = i;
		array[1][0] = j;
		return new Coordinates(array);
	}
	//ma fonction/
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
		/*
		int n = this.input.obstacles.n;
		int x, y;
		for(int p = 0; p < n ; p++) {
			x = this.input.obstacles.getX(p);
			y = this.input.obstacles.getY(p);
			couleur.get(x).put(y, "NOIR");
		}
		*/
	}
	
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
		
	public Coordinates voisin(int i, int j) {
		int[][] array;
		int x_min = this.input.xmin - MyBestAlgorithm.marge;
		int x_max = this.input.xmax + MyBestAlgorithm.marge;
		int y_min = this.input.ymin - MyBestAlgorithm.marge;
		int y_max = this.input.ymax + MyBestAlgorithm.marge;
		if(i == x_min) {
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
		int x, y, longueur = 0;
		int n = array[0].length;
		boolean[] bool = new boolean[n];
		for(int p = 0; p < n; p++) {
			x = array[0][p];
			y = array[1][p];
			
			if(libre.get(x).get(y) == 1) {
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
	// ma fonction/
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
	/**
	 * Compute a complete solution to the input problem: compute all steps, until all robots reach their target destinations
	 */
	public byte mouvement(int p, int q, Coordinates coor, int robot) {
		int n = coor.n, x, y, longueur = 0;
		int x_etoile = Integer.MAX_VALUE, y_etoile = Integer.MAX_VALUE;
		if(n == 0) {
			return Solution.FIXED;
		}else {
			int min = (int) Float.MAX_VALUE;
			for(int k = 0; k < n; k++) {
				x = coor.getX(k);
				y = coor.getY(k);
				if(positions.get(x).get(y) == 0) {
					longueur++;
					//System.out.println("robot : "+robot+", x : "+x+", y : "+y+", p : "+p+", q : "+q);
					
				}
			}
			if(longueur == 0) {return Solution.FIXED;}
			else {
				for(int k = 0; k < n; k++) {
					x = coor.getX(k);
					y = coor.getY(k);
					//System.out.println("One piece " + listeDamier.get(robot).get(x).get(y));
					if((min > listeDamier.get(robot).get(x).get(y)) && positions.get(x).get(y) == 0 ) {
						min = listeDamier.get(robot).get(x).get(y);
						System.out.println("Shame *" );
						x_etoile = x;
						y_etoile = y;
					}
										
				}
				
				
				if(x_etoile == Integer.MAX_VALUE || y_etoile == Integer.MAX_VALUE) {
					//System.out.println(n);
					//positionsCount = false;
					reInitialisation();
					m++;
//					System.out.println(current.getX(96) + "  " + current.getY(96));
					System.out.println(coor.getX(0) + ";" + coor.getY(0));
					System.out.println(coor.n +"\n");
//					return Solution.FIXED;
//					for(int k = 0; k < n; k++) {
//						x = coor.getX(k);
//						y = coor.getY(k);
						
						
//						listeDamier.get(robot).get(x).put(y, 1);
						
//						if((positions.get(x).get(y) == 0) ) {
//							//System.out.println("Here " + listeDamier.get(robot).get(x).get(y));
//							x_etoile = x;
//							y_etoile = y;
//							positionsCount = true;
//							break;
//						}
//				}
					return Solution.FIXED;
				}
				System.out.println("robot:"+robot+"x_etoile : "+x_etoile+", y_etoile : "+y_etoile);
				//if(positions.get(x_etoile).get(y_etoile) == 1 && libre.get(x_etoile).get(y_etoile)==0) {System.out.println("ICI");return Solution.FIXED;}

					listeDamier.get(robot).get(p).put(q, (int)Float.MAX_VALUE);
					if(x_etoile > p) {
						//System.out.println("Here here: " + x_etoile + "p: " + p );
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
		}
	}
	
	public void init_liste(int k) {
		int x_min = this.input.xmin - MyBestAlgorithm.marge;
		int x_max = this.input.xmax + MyBestAlgorithm.marge;
		int y_min = this.input.ymin - MyBestAlgorithm.marge;
		int y_max = this.input.ymax + MyBestAlgorithm.marge;
		listeDamier.add(k, new HashMap<Integer, HashMap<Integer, Integer>>());
		listeCouleur.add(k, new HashMap<Integer, HashMap<Integer, String>>());
		for(int i = x_min; i<= x_max; i++) {
			listeDamier.get(k).put(i, new HashMap<Integer, Integer>());
			listeCouleur.get(k).put(i, new HashMap<Integer, String>());
			for(int j = y_min; j<= y_max; j++) {
				listeDamier.get(k).get(i).put(j, damier.get(i).get(j));
				listeCouleur.get(k).get(i).put(j, couleur.get(i).get(j));
			}
		}
	}
	
	public void run() {
		// TO BE COMPLETED
		this.init_positions();
		this.initialisation();
		//System.out.println("damier de 0 : "+listeDamier.size());
		this.indices = new ArrayList<Integer>();
		for(int p = 0; p < this.input.n; p++) {
			this.indices.add(p, Integer.valueOf(p));
		}
		//System.out.println("indices : "+indices);
		///*
		while(!indices.isEmpty()){
			computeOneStep();
			//System.out.println("indices : "+indices);
			//System.out.println("target 14 : "+this.input.targets.getX(14)+"; " + this.input.targets.getY(14)) ;
		}
		//*/
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
		}
		iterateur = 0;
		while(iterateur < this.indices.size()) {
			robot = this.indices.get(iterateur);
			x = this.current.getX(robot);
			y = this.current.getY(robot);
			if((x == this.input.targets.getX(robot))&&(y == this.input.targets.getY(robot))) {
				indices.remove(Integer.valueOf(robot));
				libre.get(x).put(y, 1);
				positions.get(x).put(y, 1);
				reInitialisation();
			}else {
				
				intermediaire = voisin(x, y);
				for(int d = 0; d<intermediaire.n; d++) {
					//System.out.println("robot" + robot + " x_etoile:");
				}
				
				mouv = mouvement(x, y, intermediaire, robot);
				//System.out.println("mouv:" + mouv);
				switch(mouv) {
					case Solution.N:
						this.positions.get(x).put(y, 0);
						this.positions.get(x).put(y+1, 1);
						current.increaseY(robot);
						break;
					case Solution.S:
						this.positions.get(x).put(y, 0);
						this.positions.get(x).put(y-1, 1);
						current.decreaseY(robot);
						break;
					case Solution.E:
						this.positions.get(x).put(y, 0);
						this.positions.get(x+1).put(y, 1);
						current.increaseX(robot);
						break;
					case Solution.W:
						this.positions.get(x).put(y, 0);
						//System.out.println("x:" + x +" y_max: " + (input.ymax + 1));
						this.positions.get(x-1).put(y, 1);
						current.decreaseX(robot);
						break;
					case Solution.FIXED:
						break;
				}
				iteration[robot] = mouv;
			}
			iterateur = iterateur + 1;
			System.out.println(indices);
			
		}
		//System.out.println(this.listeDamier.get(1).get(this.input.targets.getX(1)).get(this.input.targets.getY(1)));
		this.solution.addStep(iteration);
		//throw new Error("TO BE COMPLETED");
		System.out.println(m);
	}
	
	

}

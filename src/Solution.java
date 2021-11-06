import java.util.ArrayList;

/**
 * A class defining a solution to the coordinated (robot) motion planning problem. <br>
 * 
 * The parallel-motion steps of all robots are stored in an ArrayList<byte[]>, whose 'k-th' element is an array of size 'n'.<br>
 * 
 * @author Luca Castelli Aleardi (INF421, Ecole Polytechnique, nov 2020)
 *
 */
public class Solution {
	/** Possible robot movements into Western, Eastern, Northern or Southern direction. (FIXED means that the robot is not moving) */
	public final static byte FIXED=0, N=1, S=2, E=3, W=4;

	/** Name of the input instance */
	public String name;
	
	/** 
	 * A dynamic array storing all motion parallel steps: 'k-th' element is an array of size 'n' representing
	 * the displacement of all robots at time 'k'. <br>
	 * Remark: if the 'i-th' is not moving at time 'k' then the corresponding entry in the array is FIXED (equal to 0)
	 **/
	public ArrayList<byte[]> steps;
	
	/**
	 * Initialize the motion of robots (no steps at the beginning)
	 */
	public Solution(String name) {
		this.name=name;
		this.steps=new ArrayList<byte[]>(); // empty list of steps at the beginning
	}
	
	/**
	 * Add a new step to the current solution
	 */
	public void addStep(byte[] mov) {
		if(mov!=null)
			this.steps.add(mov);
	}

	/**
	 * The makespan is just the number of parallel steps (e.g. the time until all robots have reached their destinations).
	 * 
	 * @return the makespan
	 */
	public int makespan() {
		return this.steps.size();
	}
	
	/**
	 * Compute and return the total distance traveled by all robots (until they have all reached their destinations).
	 * @return the total traveled distance of all robots (in all steps)
	 */
	public int getTotalDistance() {
		int result=0;
		for(byte[] moves: this.steps) {
			for(int i=0;i<moves.length;i++)
				if(moves[i]!=Solution.FIXED)
					result++;
		}
		return result;
	}
	
	/**
	 * Check whether the solution describe a valid trajectory for all robots and at any step.
	 * 
	 * @return TRUE is all motion parallel steps are valid (according to the rules of the problem)
	 */
	public boolean per(byte b1, byte b2) {
		boolean reponse = false;
		switch(b1) {
		case Solution.N:
			if((b2 == Solution.E)||(b2 == Solution.W)) {reponse = true;}
			break;
		case Solution.S:
			if((b2 == Solution.E)||(b2 == Solution.W)) {reponse = true;}
			break;
		case Solution.E:
			if((b2 == Solution.N)||(b2 == Solution.S)) {reponse = true;}
			break;
		case Solution.W:
			if((b2 == Solution.N)||(b2  == Solution.S)) {reponse = true;}
			break;
		}
		return reponse;
	}
	public boolean isValid(Instance instance) {
		boolean valid = true;
		int makespan = this.makespan();
		int nrobots = instance.n;
		Coordinates current = new Coordinates(instance.starts.getPositions());
		int[][] utile = new int[nrobots][nrobots];
		byte[] pas;
		int x, y;
		for(int k = 0; k < nrobots; k++) {
			for(int j = 0; j < nrobots; j++) {
				if(k==j) {utile[k][j] = 1;}else {utile[k][j] = 0;}
			}
		}
		int k = 0;
		while((k < makespan)&&(valid)) {
			pas = this.steps.get(k);
			for(int i = 0; i < nrobots; i++) {
				x = current.getX(i);
				y = current.getY(i);
				switch(pas[i]) {
					case Solution.FIXED:
						break;
					case Solution.N:
						//current.increaseY(i);
						y++;
						break;
					case Solution.S:
						//current.decreaseY(i);
						y--;
						break;
					case Solution.E:
						//current.increaseX(i);
						x++;
						break;
					case Solution.W:
						//current.decreaseX(i);
						x--;
						break;
				}
				for(int p = 0; p < nrobots; p++) {
					if(p!= i) {
						if((x == current.getX(p))&&(y == current.getY(p))&&(per(pas[i], pas[p]))) {valid = false;}
					}
				}
			}
			for(int i = 0; i < nrobots; i++) {
				switch(pas[i]) {
				case Solution.FIXED:
					break;
				case Solution.N:
					current.increaseY(i);
					break;
				case Solution.S:
					current.decreaseY(i);
					break;
				case Solution.E:
					current.increaseX(i);
					break;
				case Solution.W:
					current.decreaseX(i);
					break;
			}
			}
			for(int i = 0; i < nrobots; i++) {
				for(int j = i+1; j < nrobots; j++) {
					if((current.getX(i) == current.getX(j))&&(current.getY(i) == current.getY(j))) {utile[i][j] = 1;}else{utile[i][j] = 0;}
					if(utile[i][j] == 1) {valid = false;}
				}
				for(int p = 0; p <instance.obstacles.n; p++) {
					if((current.getX(i) == instance.obstacles.getX(p))&&(current.getY(i) == instance.obstacles.getY(p))) {valid = false;}
				}
			}
			k++;
		}
		if(valid) {
			for(int p = 0; p < nrobots; p++) {
				if((current.getX(p) == instance.targets.getX(p))&&(current.getY(p) == instance.targets.getY(p))) {}else {valid = false;}
			}
		}
		return valid;
	}
	
	public String toString() {
		String result="Solution to the input instance: "+this.name+"\n";
		result=result+"\tnumber of steps (makespan): "+this.makespan()+"\n";
		result=result+"\ttotal distance (total number of robot moves): "+this.getTotalDistance();
		return result;
	}
	
}

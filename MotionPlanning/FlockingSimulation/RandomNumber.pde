import java.util.HashSet;
public static class RandomNumbers {
  
public static void randomSet(int min, int max, int n, HashSet<Integer> set) {
    if (n > (max - min + 1) || max < min) {
        return;
    }
    for (int i = 0; i < n; i++) {
        //
        int num = (int) (Math.random() * (max - min)) + min;
        set.add(num);// 
    }
    int setSize = set.size();
   
    if (setSize < n) {
     
        randomSet(min, max, n - setSize, set); 

    }
}

}

public class PortfolioAllocation {
    
    private static final int SOURCE = 0;
    private int sinkIndex;
    private int totalNodes;
    
    private double[][] capacity;      
    private double[][] flow;         
    private int[] parent;             
    private boolean[] visited;       
    
    private String[] assets;
    private String[] sectors;
    private String[] regions;
    private double capital;
    
    private String[] nodeNames;
    
    static class Queue {
        private int[] data;
        private int front, rear, size, capacity;
        
        public Queue(int capacity) {
            this.capacity = capacity;
            this.data = new int[capacity];
            this.front = 0;
            this.rear = -1;
            this.size = 0;
        }
        
        public void enqueue(int value) {
            if (size == capacity) {
                int newCapacity = capacity * 2;
                int[] newData = new int[newCapacity];
                for (int i = 0; i < size; i++) {
                    newData[i] = data[(front + i) % capacity];
                }
                data = newData;
                front = 0;
                rear = size - 1;
                capacity = newCapacity;
            }
            rear = (rear + 1) % capacity;
            data[rear] = value;
            size++;
        }
        
        public int dequeue() {
            if (size == 0) return -1;
            int value = data[front];
            front = (front + 1) % capacity;
            size--;
            return value;
        }
        
        public boolean isEmpty() {
            return size == 0;
        }
    }
    
    public PortfolioAllocation(String[] assets, String[] sectors, 
                               String[] regions, double capital) {
        this.assets = assets;
        this.sectors = sectors;
        this.regions = regions;
        this.capital = capital;
        
        this.totalNodes = 1 + assets.length + (2 * sectors.length) + regions.length + 1;
        this.sinkIndex = totalNodes - 1;
        
        this.capacity = new double[totalNodes][totalNodes];
        this.flow = new double[totalNodes][totalNodes];
        this.parent = new int[totalNodes];
        this.visited = new boolean[totalNodes];
        
        this.nodeNames = new String[totalNodes];
        this.nodeNames[SOURCE] = "SOURCE";
        int idx = 1;
        for (int i = 0; i < assets.length; i++) {
            nodeNames[idx++] = "A_" + assets[i];
        }
        for (int i = 0; i < sectors.length; i++) {
            nodeNames[idx++] = "S_IN_" + sectors[i];
        }
        for (int i = 0; i < sectors.length; i++) {
            nodeNames[idx++] = "S_OUT_" + sectors[i];
        }
        for (int i = 0; i < regions.length; i++) {
            nodeNames[idx++] = "G_" + regions[i];
        }
        nodeNames[sinkIndex] = "SINK";
    }
    
    private int getAssetIndex(int assetId) {
        return 1 + assetId;
    }
    
    private int getSectorInIndex(int sectorId) {
        return 1 + assets.length + sectorId;
    }
    
    private int getSectorOutIndex(int sectorId) {
        return 1 + assets.length + sectors.length + sectorId;
    }
    
    private int getRegionIndex(int regionId) {
        return 1 + assets.length + (2 * sectors.length) + regionId;
    }
    
    public void buildNetwork(double[] capAsset, double[] capSector, 
                            double[] capRegion, boolean[][] compatibleAS, 
                            boolean[][] compatibleSR) {
        
        for (int i = 0; i < totalNodes; i++) {
            for (int j = 0; j < totalNodes; j++) {
                capacity[i][j] = 0.0;
            }
        }
        
        for (int i = 0; i < assets.length; i++) {
            int assetIdx = getAssetIndex(i);
            capacity[SOURCE][assetIdx] = capAsset[i] * capital;
        }
        
        for (int i = 0; i < assets.length; i++) {
            for (int j = 0; j < sectors.length; j++) {
                if (compatibleAS[i][j]) {
                    int assetIdx = getAssetIndex(i);
                    int sectorInIdx = getSectorInIndex(j);
                    capacity[assetIdx][sectorInIdx] = capital;
                }
            }
        }
        
        for (int j = 0; j < sectors.length; j++) {
            int sectorInIdx = getSectorInIndex(j);
            int sectorOutIdx = getSectorOutIndex(j);
            capacity[sectorInIdx][sectorOutIdx] = capSector[j];
        }
        
        for (int j = 0; j < sectors.length; j++) {
            for (int k = 0; k < regions.length; k++) {
                if (compatibleSR[j][k]) {
                    int sectorOutIdx = getSectorOutIndex(j);
                    int regionIdx = getRegionIndex(k);
                    capacity[sectorOutIdx][regionIdx] = capital;
                }
            }
        }
        
        for (int k = 0; k < regions.length; k++) {
            int regionIdx = getRegionIndex(k);
            capacity[regionIdx][sinkIndex] = capRegion[k] * capital;
        }
    }
    
    private boolean bfs() {
        for (int i = 0; i < totalNodes; i++) {
            visited[i] = false;
            parent[i] = -1;
        }
        
        Queue queue = new Queue(totalNodes);
        queue.enqueue(SOURCE);
        visited[SOURCE] = true;
        parent[SOURCE] = -1;
        
        while (!queue.isEmpty()) {
            int u = queue.dequeue();
            
            for (int v = 0; v < totalNodes; v++) {
                double residualCap = capacity[u][v] - flow[u][v];
                
                if (!visited[v] && residualCap > 1e-9) {
                    visited[v] = true;
                    parent[v] = u;
                    queue.enqueue(v);
                    
                    if (v == sinkIndex) {
                        return true;
                    }
                }
            }
        }
        
        return false;
    }
    
    public FlowResult edmondsKarp() {
        long startTime = System.nanoTime();
        
        // Initialize flow to 0
        for (int i = 0; i < totalNodes; i++) {
            for (int j = 0; j < totalNodes; j++) {
                flow[i][j] = 0.0;
            }
        }
        
        double maxFlow = 0.0;
        int iterations = 0;
        
        while (bfs()) {
            iterations++;
            
            double bottleneck = Double.MAX_VALUE;
            for (int v = sinkIndex; v != SOURCE; v = parent[v]) {
                int u = parent[v];
                double residualCap = capacity[u][v] - flow[u][v];
                if (residualCap < bottleneck) {
                    bottleneck = residualCap;
                }
            }
            
            for (int v = sinkIndex; v != SOURCE; v = parent[v]) {
                int u = parent[v];
                flow[u][v] += bottleneck;
                flow[v][u] -= bottleneck;
            }
            
            maxFlow += bottleneck;
        }
        
        long endTime = System.nanoTime();
        double timeTaken = (endTime - startTime) / 1_000_000.0;
        
        return new FlowResult(maxFlow, iterations, timeTaken);
    }
    
    public AggregateTotals getAggregateTotals() {
        double[] assetTotals = new double[assets.length];
        for (int i = 0; i < assets.length; i++) {
            int assetIdx = getAssetIndex(i);
            assetTotals[i] = flow[SOURCE][assetIdx];
        }
        
        double[] sectorTotals = new double[sectors.length];
        for (int j = 0; j < sectors.length; j++) {
            int sectorInIdx = getSectorInIndex(j);
            int sectorOutIdx = getSectorOutIndex(j);
            sectorTotals[j] = flow[sectorInIdx][sectorOutIdx];
        }
        
        double[] regionTotals = new double[regions.length];
        for (int k = 0; k < regions.length; k++) {
            int regionIdx = getRegionIndex(k);
            regionTotals[k] = flow[regionIdx][sinkIndex];
        }
        
        return new AggregateTotals(assetTotals, sectorTotals, regionTotals);
    }
    

    public boolean validateConstraints(FlowResult flowResult, AggregateTotals totals,
                                      double[] capAsset, double[] capSector, double[] capRegion) {
        boolean valid = true;
        double epsilon = 1e-6;
        
        if (flowResult.maxFlow > capital + epsilon) {
            System.err.println("ERROR: Allocated $" + String.format("%.2f", flowResult.maxFlow) + 
                             " exceeds capital $" + String.format("%.2f", capital));
            valid = false;
        }
        
        for (int i = 0; i < assets.length; i++) {
            double limit = capAsset[i] * capital;
            if (totals.assetTotals[i] > limit + epsilon) {
                System.err.println("ERROR: Asset " + assets[i] + " = $" + 
                                 String.format("%.2f", totals.assetTotals[i]) + 
                                 " exceeds limit $" + String.format("%.2f", limit));
                valid = false;
            }
        }
        
        for (int j = 0; j < sectors.length; j++) {
            if (totals.sectorTotals[j] > capSector[j] + epsilon) {
                System.err.println("ERROR: Sector " + sectors[j] + " = $" + 
                                 String.format("%.2f", totals.sectorTotals[j]) + 
                                 " exceeds limit $" + String.format("%.2f", capSector[j]));
                valid = false;
            }
        }
        
        for (int k = 0; k < regions.length; k++) {
            double limit = capRegion[k] * capital;
            if (totals.regionTotals[k] > limit + epsilon) {
                System.err.println("ERROR: Region " + regions[k] + " = $" + 
                                 String.format("%.2f", totals.regionTotals[k]) + 
                                 " exceeds limit $" + String.format("%.2f", limit));
                valid = false;
            }
        }
        
        if (valid) {
            System.out.println("✓ All constraints validated successfully!");
        }
        
        return valid;
    }
    
    public static boolean validateTestConstraints(double[] capAsset, double[] capSector, 
                                                   double[] capRegion, double capital) {
        boolean valid = true;
        
        double assetSum = 0.0;
        for (double cap : capAsset) {
            assetSum += cap;
        }
        
        if (assetSum > 1.0 + 1e-6) {
            System.err.println("WARNING: Asset capacity fractions sum to " + 
                             String.format("%.2f%%", assetSum * 100) + 
                             " which exceeds 100% of capital!");
            valid = false;
        }
        
        double regionSum = 0.0;
        for (double cap : capRegion) {
            regionSum += cap;
        }
        
        if (regionSum > 1.0 + 1e-6) {
            System.err.println("WARNING: Region capacity fractions sum to " + 
                             String.format("%.2f%%", regionSum * 100) + 
                             " which exceeds 100% of capital!");
            valid = false;
        }
        
        if (valid) {
            System.out.println();
        }
        
        return valid;
    }
    
    public void printResults(FlowResult flowResult, AggregateTotals totals) {
        System.out.println();
        System.out.println("                    PORTFOLIO ALLOCATION RESULTS");
        System.out.println("--------------------------------------------------------------------------------");
        System.out.println();
        
        System.out.printf("Total Capital Available:  $%,.2f%n", capital);
        System.out.printf("Total Capital Allocated:  $%,.2f%n", flowResult.maxFlow);
        System.out.printf("Utilization Rate:         %.2f%%%n", 
                         100.0 * flowResult.maxFlow / capital);
        System.out.printf("Edmonds-Karp Iterations:  %d%n", flowResult.iterations);
        System.out.printf("Computation Time:         %.3f ms%n", flowResult.timeTaken);
        System.out.println();
        
        System.out.println();
        System.out.println("ASSET CLASS TOTALS:");
        for (int i = 0; i < assets.length; i++) {
            double pct = 100.0 * totals.assetTotals[i] / capital;
            System.out.printf("  %-15s: $%,15.2f  (%.2f%% of capital)%n",
                             assets[i], totals.assetTotals[i], pct);
        }

        System.out.println();
        System.out.println("SECTOR EXPOSURE:");
        for (int j = 0; j < sectors.length; j++) {
            System.out.printf("  %-15s: $%,15.2f%n", sectors[j], totals.sectorTotals[j]);
        }

        System.out.println();
        System.out.println("GEOGRAPHIC EXPOSURE:");
        for (int k = 0; k < regions.length; k++) {
            double pct = 100.0 * totals.regionTotals[k] / capital;
            System.out.printf("  %-15s: $%,15.2f  (%.2f%% of capital)%n",
                             regions[k], totals.regionTotals[k], pct);
        }
        System.out.println();
    }
    
    public void printComplexityAnalysis() {
        int V = totalNodes;
        int E = 0;
        

        for (int i = 0; i < totalNodes; i++) {
            for (int j = 0; j < totalNodes; j++) {
                if (capacity[i][j] > 0) {
                    E++;
                }
            }
        }
        
        System.out.println();
        System.out.println("                    COMPLEXITY ANALYSIS");
        System.out.println("--------------------------------------------------------------------------------");
        System.out.println();
        
        System.out.println("GRAPH PROPERTIES:");
        System.out.printf("  Total Vertices (V):       %d%n", V);
        System.out.printf("  Total Edges (E):          %d%n", E);
        System.out.printf("  Graph Density:            %.2f%%%n", 
                         100.0 * E / (V * (V - 1)));
        System.out.println();
        
        System.out.println("TIME COMPLEXITY:");
        System.out.println("  Algorithm:                Edmonds-Karp (BFS-based Ford-Fulkerson)");
        System.out.printf("  Theoretical:              O(V * E^2) = O(%d * %d^2) = O(%,d)%n", 
                         V, E, V * E * E);
        System.out.println("  Each BFS:                 O(V + E)");
        System.out.println("  Max Augmenting Paths:     O(V * E)");
        System.out.println();
        
        System.out.println("SPACE COMPLEXITY:");
        System.out.printf("  Capacity Matrix:          O(V^2) = O(%d^2) = %,d doubles%n", 
                         V, V * V);
        System.out.printf("  Flow Matrix:              O(V^2) = O(%d^2) = %,d doubles%n", 
                         V, V * V);
        System.out.printf("  Auxiliary Arrays:         O(V) = O(%d)%n", V);
        System.out.printf("  Total Space:              O(V^2) = %,d doubles ≈ %.2f MB%n",
                         2 * V * V, (2.0 * V * V * 8) / (1024 * 1024));
        System.out.println();
    }
    
    static class FlowResult {
        double maxFlow;
        int iterations;
        double timeTaken;
        
        FlowResult(double maxFlow, int iterations, double timeTaken) {
            this.maxFlow = maxFlow;
            this.iterations = iterations;
            this.timeTaken = timeTaken;
        }
    }
    
    static class AggregateTotals {
        double[] assetTotals;
        double[] sectorTotals;
        double[] regionTotals;
        
        AggregateTotals(double[] assetTotals, double[] sectorTotals, double[] regionTotals) {
            this.assetTotals = assetTotals;
            this.sectorTotals = sectorTotals;
            this.regionTotals = regionTotals;
        }
    }
    

    public static void testCase1() {
        System.out.println("\n");
        System.out.println("                    TEST CASE 1: BASIC PORTFOLIO (4x4x4)");
        System.out.println("--------------------------------------------------------------------------------");
        System.out.println();
        
        String[] assets = {"Stocks", "Bonds", "Crypto", "RealEstate"};
        String[] sectors = {"Tech", "Healthcare", "Energy", "Finance"};
        String[] regions = {"US", "Europe", "Asia", "Emerging"};
        double capital = 100_000_000.0; // $100M
        
        double[] capAsset = {0.45, 0.25, 0.10, 0.20};
        double[] capSector = {25_000_000, 20_000_000, 15_000_000, 20_000_000};

        double[] capRegion = {0.45, 0.25, 0.20, 0.10};
        
        // Compatibility matrices
        boolean[][] compatibleAS = {
            {true, true, true, true},      // Stocks
            {false, true, true, true},     // Bonds
            {true, false, false, true},    // Crypto
            {false, true, true, true}      // RealEstate
        };
        
        boolean[][] compatibleSR = {
            {true, true, true, true},      // Tech
            {true, true, true, false},     // Healthcare
            {true, true, true, true},      // Energy
            {true, true, true, true}       // Finance
        };
        
        validateTestConstraints(capAsset, capSector, capRegion, capital);
        
        PortfolioAllocation portfolio = new PortfolioAllocation(
            assets, sectors, regions, capital);
        portfolio.buildNetwork(capAsset, capSector, capRegion, compatibleAS, compatibleSR);
        FlowResult flowResult = portfolio.edmondsKarp();
        AggregateTotals totals = portfolio.getAggregateTotals();

        portfolio.printResults(flowResult, totals);
        portfolio.validateConstraints(flowResult, totals, capAsset, capSector, capRegion);
        portfolio.printComplexityAnalysis();
    }
    
    public static void testCase2() {
        System.out.println("\n");
        System.out.println("                TEST CASE 2: MEDIUM PORTFOLIO (6x6x5)");
        System.out.println("--------------------------------------------------------------------------------");
        System.out.println();
        
        String[] assets = {"Stocks", "Bonds", "Crypto", "RealEstate", "Commodities", "Cash"};
        String[] sectors = {"Tech", "Healthcare", "Energy", "Finance", "Consumer", "Industrial"};
        String[] regions = {"US", "Europe", "Asia", "LatinAmerica", "MiddleEast"};
        double capital = 250_000_000.0;
        
        double[] capAsset = {0.40, 0.25, 0.08, 0.15, 0.10, 0.02};
        double[] capSector = {60_000_000, 45_000_000, 35_000_000, 
                              50_000_000, 40_000_000, 30_000_000};

        double[] capRegion = {0.40, 0.20, 0.25, 0.10, 0.05};

        boolean[][] compatibleAS = {
            {true, true, true, true, true, false},
            {false, true, true, true, true, true},
            {true, false, false, true, false, false},
            {false, true, true, true, false, false},
            {false, false, true, false, true, false},
            {false, false, false, true, true, true}
        };
        
        boolean[][] compatibleSR = {
            {true, true, true, true, false},
            {true, true, true, false, true},
            {true, true, true, true, true},
            {true, true, true, true, true},
            {true, true, true, true, false},
            {true, true, false, false, false}
        };
        
        validateTestConstraints(capAsset, capSector, capRegion, capital);
        
        PortfolioAllocation portfolio = new PortfolioAllocation(
            assets, sectors, regions, capital);
        portfolio.buildNetwork(capAsset, capSector, capRegion, compatibleAS, compatibleSR);
        FlowResult flowResult = portfolio.edmondsKarp();
        AggregateTotals totals = portfolio.getAggregateTotals();
        
        portfolio.printResults(flowResult, totals);
        portfolio.validateConstraints(flowResult, totals, capAsset, capSector, capRegion);
        portfolio.printComplexityAnalysis();
    }
    
    public static void testCase3() {
        System.out.println("\n");
        System.out.println("                TEST CASE 3: LARGE PORTFOLIO (8x8x6)");
        System.out.println("--------------------------------------------------------------------------------");
        System.out.println();
        
        String[] assets = {"LargeCapStocks", "SmallCapStocks", "Bonds", "Crypto", 
                          "RealEstate", "Commodities", "PrivateEquity", "Derivatives"};
        String[] sectors = {"Tech", "Healthcare", "Energy", "Finance", 
                           "Consumer", "Industrial", "Utilities", "Materials"};
        String[] regions = {"NorthAmerica", "Europe", "Asia", "LatinAmerica", 
                           "MiddleEast", "Africa"};
        double capital = 500_000_000.0; 

        double[] capAsset = {0.30, 0.15, 0.25, 0.05, 0.12, 0.08, 0.03, 0.02};
        double[] capSector = {100_000_000, 80_000_000, 60_000_000, 90_000_000,
                              70_000_000, 50_000_000, 40_000_000, 45_000_000};
        double[] capRegion = {0.35, 0.20, 0.25, 0.12, 0.05, 0.03};

        boolean[][] compatibleAS = {
            {true, true, true, true, true, false, true, false},
            {true, true, false, true, false, true, false, false},
            {false, true, true, true, true, true, false, true},
            {true, false, false, true, false, false, false, false},
            {false, true, true, true, false, false, true, false},
            {false, false, true, false, true, true, false, true},
            {true, false, false, true, false, false, true, true},
            {true, false, false, true, false, false, false, true}
        };
        
        boolean[][] compatibleSR = {
            {true, true, true, true, true, false},
            {true, true, true, false, true, true},
            {true, true, true, true, true, true},
            {true, true, true, true, true, false},
            {true, true, true, true, false, false},
            {true, true, false, false, true, true},
            {true, false, false, true, true, false},
            {true, true, true, true, false, true}
        };
        
        validateTestConstraints(capAsset, capSector, capRegion, capital);
        
        PortfolioAllocation portfolio = new PortfolioAllocation(
            assets, sectors, regions, capital);
        portfolio.buildNetwork(capAsset, capSector, capRegion, compatibleAS, compatibleSR);
        FlowResult flowResult = portfolio.edmondsKarp();
        AggregateTotals totals = portfolio.getAggregateTotals();
        
        portfolio.printResults(flowResult, totals);
        portfolio.validateConstraints(flowResult, totals, capAsset, capSector, capRegion);
        portfolio.printComplexityAnalysis();
    }

    public static void comparativeAnalysis() {
        System.out.println("\n");
        System.out.println("              COMPARATIVE ANALYSIS ACROSS ALL TEST CASES");
        System.out.println("--------------------------------------------------------------------------------");
        System.out.println();
        
        System.out.printf("%-15s %-12s %-12s %-15s %-15s%n",
                         "Test Case", "Vertices", "Edges", "Time (ms)", "Iterations");
        System.out.println("--------------------------------------------------------------------------------");

        String[][] testConfigs = {
            {"4x4x4", "4", "4", "4"},
            {"6x6x5", "6", "6", "5"},
            {"8x8x6", "8", "8", "6"}
        };
        
        for (int t = 0; t < testConfigs.length; t++) {
            String[] config = testConfigs[t];
            int nAssets = Integer.parseInt(config[1]);
            int nSectors = Integer.parseInt(config[2]);
            int nRegions = Integer.parseInt(config[3]);
            
            String[] assets = new String[nAssets];
            String[] sectors = new String[nSectors];
            String[] regions = new String[nRegions];
            
            for (int i = 0; i < nAssets; i++) assets[i] = "Asset" + i;
            for (int i = 0; i < nSectors; i++) sectors[i] = "Sector" + i;
            for (int i = 0; i < nRegions; i++) regions[i] = "Region" + i;
            
            double capital = 100_000_000.0;
            double[] capAsset = new double[nAssets];
            double[] capSector = new double[nSectors];
            double[] capRegion = new double[nRegions];
            
            for (int i = 0; i < nAssets; i++) capAsset[i] = 1.0 / nAssets;
            for (int i = 0; i < nSectors; i++) capSector[i] = 20_000_000;
            for (int i = 0; i < nRegions; i++) capRegion[i] = 1.0 / nRegions;
            
            boolean[][] compatibleAS = new boolean[nAssets][nSectors];
            boolean[][] compatibleSR = new boolean[nSectors][nRegions];
            
            for (int i = 0; i < nAssets; i++) {
                for (int j = 0; j < nSectors; j++) {
                    compatibleAS[i][j] = (i + j) % 3 != 0;
                }
            }
            
            for (int i = 0; i < nSectors; i++) {
                for (int j = 0; j < nRegions; j++) {
                    compatibleSR[i][j] = (i + j) % 3 != 0;
                }
            }
            
            PortfolioAllocation portfolio = new PortfolioAllocation(
                assets, sectors, regions, capital);
            portfolio.buildNetwork(capAsset, capSector, capRegion, compatibleAS, compatibleSR);
            FlowResult result = portfolio.edmondsKarp();
            
            int V = portfolio.totalNodes;
            int E = 0;
            for (int i = 0; i < V; i++) {
                for (int j = 0; j < V; j++) {
                    if (portfolio.capacity[i][j] > 0) E++;
                }
            }
            
            System.out.printf("%-15s %-12d %-12d %-15.3f %-15d%n",
                             config[0], V, E, result.timeTaken, result.iterations);
        }
    }
    
    public static void main(String[] args) {
        System.out.println();

        for (int i = 0; i < 3; i++) {
            String[] assets = {"A1", "A2"};
            String[] sectors = {"S1", "S2"};
            String[] regions = {"R1", "R2"};
            PortfolioAllocation warmup = new PortfolioAllocation(assets, sectors, regions, 1000000);
            double[] capA = {0.5, 0.5};
            double[] capS = {500000, 500000};
            double[] capR = {0.5, 0.5};
            boolean[][] compAS = {{true, true}, {true, true}};
            boolean[][] compSR = {{true, true}, {true, true}};
            warmup.buildNetwork(capA, capS, capR, compAS, compSR);
            warmup.edmondsKarp();
        }
        
        
        testCase1();
        testCase2();
        testCase3();

        comparativeAnalysis();
        
        System.out.println();
    }
}

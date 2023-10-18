

#define PRX_DEBUG_VAR_1(VAR) std::cout << #VAR << ": " << VAR << std::endl;
#define PRX_DEBUG_VAR_2(VAR1, VAR2) std::cout << #VAR1 << ": " << VAR1 << "\t" << #VAR2 << ": " << VAR2 << std::endl;
#define PRX_DEBUG_VAR_3(VAR1, VAR2, VAR3)                                                                              \
  std::cout << #VAR1 << ": " << VAR1 << "\t" << #VAR2 << ": " << VAR2 << "\t" << #VAR3 << ": " << VAR3 << std::endl;

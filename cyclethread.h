class cyclethread {
public:
	void cycle_start_process();
	void measureover_process();
	void cycle_over_process();
private:
	bool cycleended;
	bool measureover,cycleend,SingelOver;
};
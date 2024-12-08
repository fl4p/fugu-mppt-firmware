#include "rt.h"


esp_err_t esp_intr_dump(FILE *stream) {
    if (stream == NULL) {
        stream = stdout;
    }
#ifdef CONFIG_ESP_SYSTEM_SINGLE_CORE_MODE
    const int cpu_num = 1;
#else
    const int cpu_num = SOC_CPU_CORES_NUM;
#endif

    int general_use_ints_free = 0;
    int shared_ints = 0;

    for (int cpu = 0; cpu < cpu_num; ++cpu) {
        fprintf(stream, "CPU %d interrupt status:\n", cpu);
        fprintf(stream, " Int  Level  Type   Status\n");
        for (int i_num = 0; i_num < CPU_INT_LINES_COUNT; ++i_num) {
            fprintf(stream, " %2d  ", i_num);
            esp_cpu_intr_desc_t intr_desc;
            esp_cpu_intr_get_desc(cpu, i_num, &intr_desc);
            bool is_general_use = true;
            vector_desc_t *vd = find_desc_for_int(i_num, cpu);

#ifndef SOC_CPU_HAS_FLEXIBLE_INTC
            fprintf(stream, "   %d    %s  ",
                    intr_desc.priority,
                    intr_desc.type == ESP_CPU_INTR_TYPE_EDGE ? "Edge " : "Level");

            is_general_use = (intr_desc.type == ESP_CPU_INTR_TYPE_LEVEL) && (intr_desc.priority <= XCHAL_EXCM_LEVEL);
#else // SOC_CPU_HAS_FLEXIBLE_INTC
            if (vd == NULL) {
                fprintf(stream, "   *      *    ");
            } else {
                // # TODO: IDF-9512
                // esp_cpu_intr_get_* functions need to be extended with cpu parameter.
                // Showing info for the current cpu only, in the meantime.
                if (esp_cpu_get_core_id() == cpu) {
                    fprintf(stream, "   %d    %s  ",
                            esp_cpu_intr_get_priority(i_num),
                            esp_cpu_intr_get_type(i_num) == ESP_CPU_INTR_TYPE_EDGE ? "Edge " : "Level");
                } else {
                    fprintf(stream, "   ?      ?    ");
                }
            }
#endif // SOC_CPU_HAS_FLEXIBLE_INTC

            if (intr_desc.flags & ESP_CPU_INTR_DESC_FLAG_RESVD) {
                fprintf(stream, "Reserved");
            } else if (intr_desc.flags & ESP_CPU_INTR_DESC_FLAG_SPECIAL) {
                fprintf(stream, "CPU-internal");
            } else {
                if (vd == NULL || (vd->flags & (VECDESC_FL_RESERVED | VECDESC_FL_NONSHARED | VECDESC_FL_SHARED)) == 0) {
                    fprintf(stream, "Free");
                    if (is_general_use) {
                        ++general_use_ints_free;
                    } else {
                        fprintf(stream, " (not general-use)");
                    }
                } else if (vd->flags & VECDESC_FL_RESERVED) {
                    fprintf(stream, "Reserved (run-time)");
                } else if (vd->flags & VECDESC_FL_NONSHARED) {
                    fprintf(stream, "Used: %s", esp_isr_names[vd->source]);
                } else if (vd->flags & VECDESC_FL_SHARED) {
                    fprintf(stream, "Shared: ");
                    for (shared_vector_desc_t *svd = vd->shared_vec_info; svd != NULL; svd = svd->next) {
                        fprintf(stream, "%s ", esp_isr_names[svd->source]);
                    }
                    ++shared_ints;
                } else {
                    fprintf(stream, "Unknown, flags = 0x%x", vd->flags);
                }
            }

            fprintf(stream, "\n");
        }
    }
    fprintf(stream, "Interrupts available for general use: %d\n", general_use_ints_free);
    fprintf(stream, "Shared interrupts: %d\n", shared_ints);
    return ESP_OK;
}



void rtcount(const char *l) {
    static unsigned long t0 = 0;

    if (rtcount_en && t0) {
        //constexpr auto maxT = std::numeric_limits<unsigned long>::max();
        auto t = rtclock_us();
        //auto dt = (t < t0) ? (maxT - t0 + t) : (t - t0);
        auto dt = (t - t0) / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
        //if(dt > maxT/2) dt = maxT - dt;
        auto f = rtcount_stats.find(l);
        if (f == rtcount_stats.end()) {
            rtcount_stats[l] = {};
            f = rtcount_stats.find(l);
        }
        auto &stat(f->second);
        stat.total += dt;
        if (dt > stat.max) {
            stat.max = dt;
            stat.max_num = stat.num;
        }
        if (dt < stat.min) {
            stat.min = dt;
            stat.min_num = stat.num;
        }
        ++stat.num;
    }

    t0 = rtclock_us();
}

 void rtcount_print(bool reset) {
    if (rtcount_en) {
        rtcount_en = false;
        vTaskDelay(100);
    }

    printf("rtcount_print :\n");
    printf("%-30s %9s %9s %6s %6s %6s %6s %6s\n", "key", "num", "tot", "mean", "max", "maxNum", "min", "minNum");

    //typedef std::remove_reference<decltype(*rtcount_stats.begin())>::type P;
    typedef std::pair<const char *, rtcount_stat> P;
    std::vector<P> sorted(rtcount_stats.begin(), rtcount_stats.end());
    std::sort(sorted.begin(), sorted.end(), [](const P &a, const P &b) {
        return a.second.max > b.second.max;
    });

    for (auto [k, stat]: sorted) {
        printf("%-30s %9lu %9lu %6lu %6lu %6lu %6lu %6lu\n", k, stat.num, stat.total, stat.total / stat.num,
               stat.max, stat.max_num, stat.min, stat.min_num);
    }
    printf("\n\n");
    if (reset)
        rtcount_stats.clear();

    rtcount_en = true;
}

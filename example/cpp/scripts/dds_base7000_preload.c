#define _GNU_SOURCE
#include <dds/dds.h>
#include <dlfcn.h>

static dds_entity_t (*real_create_domain)(dds_domainid_t, const char *);

dds_entity_t dds_create_domain(dds_domainid_t domain, const char *config)
{
    if (!real_create_domain)
        real_create_domain = (dds_entity_t (*)(dds_domainid_t, const char *))
            dlsym(RTLD_NEXT, "dds_create_domain");
    const char *xml =
        "<?xml version=\"1.0\"?><CycloneDDS xmlns=\"https://cdds.io/config\">"
        "<Domain id=\"any\"><General><Interfaces>"
        "<NetworkInterface name=\"lo\" multicast=\"default\"/>"
        "</Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex>"
        "<MaxAutoParticipantIndex>9</MaxAutoParticipantIndex>"
        "<Ports><Base>7000</Base></Ports></Discovery></Domain></CycloneDDS>";
    (void)config;
    return real_create_domain(domain, xml);
}
